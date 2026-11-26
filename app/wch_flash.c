/*
 * CH32 flash programming through the RISC-V debug module.
 *
 * CH32V2xx/3xx: 256-byte fast pages (FTER + FTPG/PGSTRT), with HCLK limited
 * to <= 60 MHz for the session when the target runs from the PLL.
 *
 * CH32V003 (family 0x09): 64-byte pages; program with FTPG, BUF_RST, per-word
 * BUF_LOAD, then FTPG|STRT (bit 6 — not PGSTRT). No AHB clock tweak.
 *
 * Hosts send the image padded to the page size. The flash routine they
 * upload (02 05) is discarded; the probe programs through the DM.
 *
 * The probe acknowledges every write-pack and the end with 41 01 01 04 on
 * EP 0x82 (wlink: 1024 B for V003/V00X, 4096 B otherwise).
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>
#include "wch_flash.h"
#include "rvdm.h"
#include "hal.h"

#define RCC_CFGR0       0x40021004u
#define SWS_MASK        (3u << 2)
#define SWS_PLL         (2u << 2)
#define HPRE_MASK       (0xFu << 4)
#define HPRE_DIV4       (0x9u << 4)

#define FLASH_BASE      0x40022000u
#define FLASH_KEYR      (FLASH_BASE + 0x04)
#define FLASH_OBKEYR    (FLASH_BASE + 0x08)
#define FLASH_STATR     (FLASH_BASE + 0x0C)
#define FLASH_CTLR      (FLASH_BASE + 0x10)
#define FLASH_ADDR      (FLASH_BASE + 0x14)
#define FLASH_OBR       (FLASH_BASE + 0x1C)
#define FLASH_MODEKEYR  (FLASH_BASE + 0x24)

#define KEY1            0x45670123u
#define KEY2            0xCDEF89ABu

#define CTLR_PG         (1u << 0)
#define CTLR_MER        (1u << 2)
#define CTLR_OBPG       (1u << 4)
#define CTLR_OBER       (1u << 5)
#define CTLR_STRT       (1u << 6)
#define CTLR_LOCK       (1u << 7)
#define CTLR_OBWRE      (1u << 9)
#define CTLR_FLOCK      (1u << 15)
#define CTLR_FTPG       (1u << 16)
#define CTLR_FTER       (1u << 17)
#define CTLR_BUFLOAD    (1u << 18)
#define CTLR_BUFRST     (1u << 19)
#define CTLR_PGSTRT     (1u << 21)

#define STATR_BSY       (1u << 0)
#define STATR_WRBSY     (1u << 1)
#define STATR_WRPRTERR  (1u << 4)
#define STATR_EOP       (1u << 5)

#define OB_RDPR         0x1FFFF800u
#define RDPR_UNPROTECTED 0x00A5u

#define PAGE_V3         256u
#define PAGE_V003       64u
#define PAGE_MAX        PAGE_V3
#define ACK_V3          4096u
#define ACK_V003        1024u   /* wlink write_pack_size for CH32V003 */
#define ROUTINE_SIZE    512u
#define FAMILY_V003     0x09u

static enum { IDLE, ROUTINE, WRITING } state;
static uint32_t routine_left;
static uint32_t base, total, received, acked_at;
static uint8_t page[PAGE_MAX] __attribute__((aligned(4)));
static uint32_t fill;
static bool session, was_running, ack_due, failed, verify;
static uint32_t saved_cfgr0;
static uint8_t family = 0x06;
static uint32_t page_size = PAGE_V3;

void wch_flash_init(void)
{
    state = IDLE;
    session = false;
    family = 0x06;
    page_size = PAGE_V3;
}

void wch_flash_set_family(uint8_t f)
{
    family = f;
    page_size = (f == FAMILY_V003) ? PAGE_V003 : PAGE_V3;
}

static bool is_v003(void)
{
    return family == FAMILY_V003;
}

static bool rd(uint32_t a, uint32_t *v) { return rvdm_mem_read32(a, v); }
static bool wr(uint32_t a, uint32_t v) { return rvdm_mem_write32(a, v); }

/* SDI mis-reads often yield 0 / 0xffffffff — treat as dead, do not spin 2 s. */
static bool statr_sane(uint32_t s)
{
    return s != 0u && s != 0xffffffffu;
}

static bool wait_idle(void)
{
    uint32_t t = hal_millis(), s;
    do {
        if (!rd(FLASH_STATR, &s) || !statr_sane(s))
            return false;
        if (!(s & STATR_BSY)) {
            if (s & (STATR_EOP | STATR_WRPRTERR))
                wr(FLASH_STATR, STATR_EOP | STATR_WRPRTERR);
            return !(s & STATR_WRPRTERR);
        }
    } while (hal_millis() - t < 500u);
    return false;
}

static bool wait_bsy(void)
{
    uint32_t t = hal_millis(), s;
    do {
        if (!rd(FLASH_STATR, &s) || !statr_sane(s))
            return false;
        if (!(s & STATR_BSY))
            return true;
    } while (hal_millis() - t < 500u);
    return false;
}

static bool open_session(void)
{
    if (session)
        return true;
    if (!rvdm_begin(&was_running))
        return false;
    session = true;
    uint32_t c;
    saved_cfgr0 = 0xFFFFFFFFu;
    if (!is_v003() && rd(RCC_CFGR0, &saved_cfgr0)) {
        /* only when running from the PLL: the debug interface runs on HCLK
         * too, and slowing an 8 MHz HSI clock down breaks the link */
        uint32_t h = saved_cfgr0 & HPRE_MASK;
        if ((saved_cfgr0 & SWS_MASK) == SWS_PLL && h < HPRE_DIV4)
            wr(RCC_CFGR0, (saved_cfgr0 & ~HPRE_MASK) | HPRE_DIV4);
        else
            saved_cfgr0 = 0xFFFFFFFFu;
    }
    wr(FLASH_KEYR, KEY1);
    wr(FLASH_KEYR, KEY2);
    wr(FLASH_OBKEYR, KEY1);
    wr(FLASH_OBKEYR, KEY2);
    wr(FLASH_MODEKEYR, KEY1);       /* fast mode */
    wr(FLASH_MODEKEYR, KEY2);
    return rd(FLASH_CTLR, &c) && !(c & (CTLR_LOCK | CTLR_FLOCK));
}

static void close_session_halted(bool stay_halted)
{
    if (!session)
        return;
    wr(FLASH_CTLR, CTLR_LOCK | CTLR_FLOCK);
    if (!is_v003() && saved_cfgr0 != 0xFFFFFFFFu)
        wr(RCC_CFGR0, saved_cfgr0);
    rvdm_end(was_running && !stay_halted);
    session = false;
}

static void close_session(void)
{
    close_session_halted(false);
}

static bool ctlr_op(uint32_t bits)
{
    return wr(FLASH_CTLR, bits) && wr(FLASH_CTLR, bits | CTLR_STRT) &&
           wait_idle() && wr(FLASH_CTLR, 0);
}

bool wch_flash_mass_erase(void)
{
    bool ok = open_session() && ctlr_op(CTLR_MER);
    close_session_halted(true);         /* never resume into erased flash */
    return ok;
}

static bool write_page_v3(uint32_t addr, const uint8_t *p)
{
    /* fast page erase */
    if (!wr(FLASH_CTLR, CTLR_FTER) || !wr(FLASH_ADDR, addr) ||
        !wr(FLASH_CTLR, CTLR_FTER | CTLR_STRT) || !wait_idle() ||
        !wr(FLASH_CTLR, 0))
        return false;
    /* fast page program: 64 words into the page buffer (BSY stays set,
     * WRBSY gates each word), then PGSTRT */
    bool ok = wr(FLASH_CTLR, CTLR_FTPG) &&
              rvdm_mem_stream32(addr, p, page_size / 4u, FLASH_STATR, STATR_WRBSY) &&
              wr(FLASH_CTLR, CTLR_FTPG | CTLR_PGSTRT) && wait_idle();
    wr(FLASH_CTLR, 0);
    return ok;
}

/* V003: FTER; FTPG|BUFRST; BSY; ADDR; 16×(word,BUFLOAD,BSY); FTPG|STRT. */
static bool write_page_v003(uint32_t addr, const uint8_t *p)
{
    if (!wr(FLASH_CTLR, CTLR_FTER) || !wr(FLASH_ADDR, addr) ||
        !wr(FLASH_CTLR, CTLR_FTER | CTLR_STRT) || !wait_idle() ||
        !wr(FLASH_CTLR, 0))
        return false;
    if (!wr(FLASH_CTLR, CTLR_FTPG) ||
        !wr(FLASH_CTLR, CTLR_FTPG | CTLR_BUFRST) ||
        !wait_bsy() ||
        !wr(FLASH_ADDR, addr))
        return false;
    for (uint32_t i = 0; i < page_size; i += 4) {
        uint32_t w;
        memcpy(&w, &p[i], 4);
        if (!wr(addr + i, w) ||
            !wr(FLASH_CTLR, CTLR_FTPG | CTLR_BUFLOAD) ||
            !wait_bsy())
            return false;
    }
    bool ok = wr(FLASH_CTLR, CTLR_FTPG | CTLR_STRT) && wait_idle();
    wr(FLASH_CTLR, 0);
    return ok;
}

static bool write_page(uint32_t addr, const uint8_t *p)
{
    return is_v003() ? write_page_v003(addr, p) : write_page_v3(addr, p);
}

static bool verify_page(uint32_t addr, const uint8_t *p)
{
    for (uint32_t i = 0; i < page_size; i += 4) {
        uint32_t w, v;
        memcpy(&v, &p[i], 4);
        if (!rd(addr + i, &w) || w != v)
            return false;
    }
    return true;
}

/* Option bytes: 8 halfwords from 0x1FFFF800 (RDPR, USER, DATA0/1,
 * WRPR0..3), each a byte plus its complement. They are read, erased and
 * written back with only RDPR changed: 0xA5 = unprotected, left erased =
 * protected. OBWRE (set by the OBKEYR unlock) is kept in every CTLR write:
 * writing it as 0 drops the option byte write access. */
#define OB_COUNT 8

static bool option_bytes(bool unprotect)
{
    uint16_t ob[OB_COUNT];
    if (!open_session())
        return false;
    for (unsigned i = 0; i < OB_COUNT; i += 2) {
        uint32_t w;
        if (!rd(OB_RDPR + 2 * i, &w)) {
            close_session();
            return false;
        }
        ob[i] = (uint16_t)w;
        ob[i + 1] = (uint16_t)(w >> 16);
    }
    wr(FLASH_OBKEYR, KEY1);
    wr(FLASH_OBKEYR, KEY2);
    bool ok = wr(FLASH_CTLR, CTLR_OBER | CTLR_OBWRE) &&
              wr(FLASH_CTLR, CTLR_OBER | CTLR_OBWRE | CTLR_STRT) && wait_idle() &&
              wr(FLASH_CTLR, CTLR_OBPG | CTLR_OBWRE);
    for (unsigned i = 0; ok && i < OB_COUNT; i++) {
        uint16_t v;
        if (i == 0) {
            if (!unprotect)
                continue;                   /* erased RDPR: protected */
            v = RDPR_UNPROTECTED;
        } else {
            /* a valid byte/complement pair is written back, an erased or
             * broken one is left erased (its default) */
            if (((ob[i] ^ (ob[i] >> 8)) & 0xFFu) != 0xFFu)
                continue;
            v = ob[i] & 0xFFu;
        }
        ok = rvdm_mem_write16(OB_RDPR + 2u * i, v) && wait_idle();
    }
    wr(FLASH_CTLR, 0);
    close_session();
    return ok;
}

bool wch_flash_protect(void)
{
    return option_bytes(false);
}

bool wch_flash_unprotect(void)
{
    return option_bytes(true);
}

void wch_flash_routine_upload(void)
{
    state = ROUTINE;
    routine_left = ROUTINE_SIZE;
}

void wch_flash_write_start(uint32_t addr, uint32_t len, bool check)
{
    uint32_t mask = page_size - 1u;
    if (state == WRITING && received == 0 && base == (addr & ~mask)) {
        verify |= check;                /* 02 07 followed by 02 02 / 02 04 */
        return;
    }
    verify = check;
    base = addr & ~mask;
    total = ((addr - base) + len + mask) & ~mask;
    received = acked_at = fill = 0;
    ack_due = failed = false;
    state = total ? WRITING : IDLE;
    if (state == WRITING && !open_session())
        failed = true;
}

void wch_flash_write_end(void)
{
    state = IDLE;
    close_session();
}

bool wch_flash_wants_data(void)
{
    return state == ROUTINE || (state == WRITING && !ack_due);
}

void wch_flash_data(const uint8_t *buf, uint16_t len)
{
    if (state == ROUTINE) {
        routine_left = len >= routine_left ? 0 : routine_left - len;
        if (!routine_left)
            state = IDLE;
        return;
    }
    if (state != WRITING)
        return;
    while (len) {
        uint32_t n = page_size - fill;
        if (n > len)
            n = len;
        memcpy(&page[fill], buf, n);
        fill += n;
        buf += n;
        len -= n;
        received += n;
        if (fill == page_size) {
            uint32_t a = base + received - page_size;
            if (!failed &&
                (!write_page(a, page) || (verify && !verify_page(a, page)))) {
                failed = true;
                close_session();
            }
            fill = 0;
        }
    }
    /* no ack on failure → host times out (genuine link error path unknown) */
    {
        uint32_t every = is_v003() ? ACK_V003 : ACK_V3;
        if (!failed && (received - acked_at >= every || received >= total)) {
            acked_at = received;
            ack_due = true;
        }
    }
}

bool wch_flash_ack_pending(uint8_t ack[4])
{
    if (!ack_due)
        return false;
    ack[0] = 0x41; ack[1] = 0x01; ack[2] = 0x01; ack[3] = 0x04;
    return true;
}

void wch_flash_ack_sent(void)
{
    ack_due = false;
    if (state == WRITING && received >= total) {
        state = IDLE;
        close_session();                /* MounRiver sends no 02 08 */
    }
}
