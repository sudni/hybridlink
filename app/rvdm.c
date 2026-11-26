/*
 * RISC-V debug module helpers (debug spec 0.13), written from the spec.
 * Memory goes through the program buffer: s0 holds the address, s1 the
 * data; both are saved and restored around a series of accesses.
 * DMI rides either two-wire RVSWD or one-wire SDI.
 * SPDX-License-Identifier: MIT
 */
#include "rvdm.h"
#include "rvswd.h"
#include "sdi.h"
#include "hal.h"

#define ABSTRACTCS_BUSY   (1u << 12)
#define ABSTRACTCS_CMDERR (7u << 8)

/* access register: aarsize 32 bit, transfer, [write], [postexec] */
#define CMD_REG       ((2u << 20) | (1u << 17))
#define CMD_WRITE     (1u << 16)
#define CMD_POSTEXEC  (1u << 18)
#define REG_S0        0x1008u
#define REG_S1        0x1009u

#define INSN_LW_S1_S0   0x00042483u   /* lw   s1, 0(s0) */
#define INSN_SW_S1_S0   0x00942023u   /* sw   s1, 0(s0) */
#define INSN_SH_S1_S0   0x00941023u   /* sh   s1, 0(s0) */
#define INSN_EBREAK     0x00100073u

static uint32_t saved_s0, saved_s1;
static uint32_t progbuf_op;             /* instruction now in progbuf0 */
static rvdm_phy_t phy = RVDM_PHY_RVSWD;
static uint32_t attach_cap, attach_status, attach_id;
static uint8_t attach_phy;

void rvdm_set_phy(rvdm_phy_t p)
{
    phy = p;
}

rvdm_phy_t rvdm_phy(void)
{
    return phy;
}

void rvdm_set_delay(uint32_t d)
{
    rvswd_set_delay(d);
    sdi_set_delay(d);
}

uint8_t rvdm_read(uint8_t addr, uint32_t *val)
{
    return phy == RVDM_PHY_SDI ? sdi_read(addr, val) : rvswd_read(addr, val);
}

uint8_t rvdm_write(uint8_t addr, uint32_t val)
{
    return phy == RVDM_PHY_SDI ? sdi_write(addr, val) : rvswd_write(addr, val);
}

static bool wait_status_ms(uint32_t mask, uint32_t ms)
{
    uint32_t t = hal_millis(), s;
    do {
        if (rvdm_read(DM_DMSTATUS, &s) != DMI_OK)
            return false;
        if (s & mask)
            return true;
    } while (hal_millis() - t < ms);
    return false;
}

static bool wait_status(uint32_t mask)
{
    return wait_status_ms(mask, 100u);
}

static void phy_init(rvdm_phy_t p)
{
    phy = p;
    if (p == RVDM_PHY_SDI) {
        sdi_init();
        /* sdi_wake() runs per pinout attempt inside try_attach */
    } else {
        rvswd_init();
        rvswd_wake();
    }
}

void rvdm_attach_probe(uint32_t *cap, uint32_t *status, uint32_t *id,
                       uint8_t *phy_used)
{
    if (cap)
        *cap = attach_cap;
    if (status)
        *status = attach_status;
    if (id)
        *id = attach_id;
    if (phy_used)
        *phy_used = attach_phy;
}

static bool try_attach_sdi_once(void)
{
    /* funprog InitializeSWDSWIO: unlock twice, dmactive twice, check CFGR */
    rvdm_write(DM_WCH_SHDWCFGR, DM_WCH_OUTEN);
    rvdm_write(DM_WCH_CFGR, DM_WCH_OUTEN);
    rvdm_write(DM_WCH_SHDWCFGR, DM_WCH_OUTEN);
    rvdm_write(DM_WCH_CFGR, DM_WCH_OUTEN);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE);
    rvdm_read(DM_WCH_CPBR, &attach_cap);
    if (attach_cap == 0x00010403u)
        return true;
    /* Some parts answer CFGR/CHIPID before CAP looks right */
    {
        uint32_t cfgr = 0xffffffffu, id = 0xffffffffu;
        rvdm_read(DM_WCH_CFGR, &cfgr);
        rvdm_read(DM_WCH_CHIPID, &id);
        if (cfgr != 0xffffffffu && cfgr != 0u)
            attach_status = cfgr; /* stash for 0x55 diag if we still fail */
        if ((id >> 20) == 0x003u) {
            attach_id = id;
            attach_cap = (attach_cap == 0xffffffffu) ? 0x00010403u : attach_cap;
            return true;
        }
    }
    return false;
}

static bool try_attach(uint32_t *chip_id)
{
    attach_phy = (uint8_t)phy;
    attach_cap = attach_status = attach_id = 0xffffffffu;

    if (phy == RVDM_PHY_SDI) {
        /* Pass 0: bus reset only (LinkE often needs no NRST).
         * Pass 1: hold NRST (PD1 lockout).
         * Pass 2: release NRST and race remux. */
        uint8_t n = sdi_preset_count();
        uint32_t best_cap = 0xffffffffu;
        /* SWDIO only — swap polluted STATUS when timing returned ff. */
        sdi_set_pinout(0);
        for (uint8_t pass = 0; pass < 2; pass++) {
            if (pass == 0)
                sdi_wake();
            else
                sdi_wake_run();
            for (uint8_t i = 0; i < n; i++) {
                sdi_set_preset(i);
                if (i) {
                    sdi_write(DM_WCH_SHDWCFGR, 0);
                    hal_delay_us(50);
                }
                if (try_attach_sdi_once())
                    goto sdi_ok;
                if (attach_cap != 0xffffffffu && attach_cap != 0u)
                    best_cap = attach_cap;
            }
        }
        if (best_cap != 0xffffffffu)
            attach_cap = best_cap;
        attach_status = sdi_line_probe();
        if (attach_id == 0xffffffffu)
            rvdm_read(DM_WCH_CHIPID, &attach_id);
        return false;
    sdi_ok:
        sdi_leave_reset();
    } else {
        rvdm_write(DM_DMCONTROL, 0);
        rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE);
    }
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_HALTREQ);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_HALTREQ);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_HALTREQ);
    if (rvdm_read(DM_DMSTATUS, &attach_status) != DMI_OK ||
        attach_status == 0u || attach_status == 0xffffffffu)
        return false;
    progbuf_op = 0;
    if (rvdm_read(DM_WCH_CHIPID, &attach_id) != DMI_OK)
        return false;
    if (attach_id == 0u || attach_id == 0xffffffffu)
        return false;
    /* V003 chip id is 0x003xxxxx; reject obvious RVSWD false positives */
    if (phy == RVDM_PHY_SDI && (attach_id >> 20) != 0x003u)
        return false;
    *chip_id = attach_id;
    /* short wait only — wlink USB I/O times out at 100 ms */
    if (!(attach_status & DMSTATUS_ALLHALTED))
        wait_status_ms(DMSTATUS_ALLHALTED, 20u);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE);
    return true;
}

bool rvdm_attach(uint32_t *chip_id, uint8_t family_hint)
{
    /*
     * wlink --chip CH32V003 sends family 0x09: SDI only (do not fall back to
     * RVSWD — bit-banging CLK on a live SWIO line can false-OK and then
     * stall in halt for 100 ms past the host USB timeout).
     * probe-rs still defaults to 0x01: try SDI then RVSWD.
     */
    rvdm_phy_t order[2];
    unsigned n = 2;
    if (family_hint == 0x09) {
        order[0] = RVDM_PHY_SDI;
        n = 1;
    } else if (family_hint == 0x05 || family_hint == 0x06) {
        order[0] = RVDM_PHY_RVSWD;
        order[1] = RVDM_PHY_SDI;
    } else {
        order[0] = RVDM_PHY_SDI;
        order[1] = RVDM_PHY_RVSWD;
    }
    for (unsigned i = 0; i < n; i++) {
        phy_init(order[i]);
        if (try_attach(chip_id))
            return true;
    }
    return false;
}

bool rvdm_is_halted(void)
{
    uint32_t s;
    return rvdm_read(DM_DMSTATUS, &s) == DMI_OK && (s & DMSTATUS_ALLHALTED);
}

bool rvdm_halt(void)
{
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_HALTREQ);
    bool ok = wait_status(DMSTATUS_ALLHALTED);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE);
    return ok;
}

bool rvdm_resume(void)
{
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_RESUMEREQ);
    bool ok = wait_status(DMSTATUS_ALLRESUMEACK);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE);
    return ok;
}

bool rvdm_reset(bool halt)
{
    uint32_t h = halt ? DMCONTROL_HALTREQ : 0;
    /* a DM cycle first: frees an abstract command stuck on a bus access */
    rvdm_write(DM_DMCONTROL, 0);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_NDMRESET | h);
    hal_delay_ms(2);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE | h);
    bool ok = halt ? wait_status(DMSTATUS_ALLHALTED) : true;
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_ACKHAVERESET);
    rvdm_write(DM_DMCONTROL, DMCONTROL_DMACTIVE);
    progbuf_op = 0;
    return ok;
}

static bool command(uint32_t cmd)
{
    uint32_t cs;
    if (rvdm_write(DM_COMMAND, cmd) != DMI_OK)
        return false;
    for (int i = 0; i < 100; i++) {
        if (rvdm_read(DM_ABSTRACTCS, &cs) != DMI_OK)
            return false;
        if (!(cs & ABSTRACTCS_BUSY))
            break;
    }
    if (cs & (ABSTRACTCS_BUSY | ABSTRACTCS_CMDERR)) {
        rvdm_write(DM_ABSTRACTCS, ABSTRACTCS_CMDERR);
        return false;
    }
    return true;
}

static bool reg_read(uint32_t reg, uint32_t *v)
{
    return command(CMD_REG | reg) && rvdm_read(DM_DATA0, v) == DMI_OK;
}

static bool reg_write(uint32_t reg, uint32_t v, bool exec)
{
    return rvdm_write(DM_DATA0, v) == DMI_OK &&
           command(CMD_REG | CMD_WRITE | (exec ? CMD_POSTEXEC : 0) | reg);
}

static bool progbuf(uint32_t insn)
{
    if (progbuf_op == insn)
        return true;
    if (rvdm_write(DM_PROGBUF0, insn) != DMI_OK ||
        rvdm_write(DM_PROGBUF0 + 1, INSN_EBREAK) != DMI_OK)
        return false;
    progbuf_op = insn;
    return true;
}

bool rvdm_begin(bool *was_running)
{
    *was_running = !rvdm_is_halted();
    if (*was_running && !rvdm_halt())
        return false;
    progbuf_op = 0;
    rvdm_write(DM_ABSTRACTCS, ABSTRACTCS_CMDERR);   /* sticky, maybe left by a host */
    if (reg_read(REG_S0, &saved_s0) && reg_read(REG_S1, &saved_s1))
        return true;
    if (*was_running)
        rvdm_resume();
    return false;
}

void rvdm_end(bool was_running)
{
    reg_write(REG_S0, saved_s0, false);
    reg_write(REG_S1, saved_s1, false);
    if (was_running)
        rvdm_resume();
}

bool rvdm_mem_read32(uint32_t addr, uint32_t *val)
{
    return progbuf(INSN_LW_S1_S0) && reg_write(REG_S0, addr, true) &&
           reg_read(REG_S1, val);
}

bool rvdm_mem_write32(uint32_t addr, uint32_t val)
{
    return progbuf(INSN_SW_S1_S0) && reg_write(REG_S1, val, false) &&
           reg_write(REG_S0, addr, true);
}

bool rvdm_mem_write16(uint32_t addr, uint16_t val)
{
    return progbuf(INSN_SH_S1_S0) && reg_write(REG_S1, val, false) &&
           reg_write(REG_S0, addr, true);
}

/* store n words from p (little endian) at addr, for flash programming.
 * The program buffer stores s1, advances s0 and then spins until the bits
 * in mask of the status register at busy_reg read 0. Per word: two DMI
 * writes plus the abstract command status (a write while the command is
 * still busy would be dropped and set a sticky error). */
bool rvdm_mem_stream32(uint32_t addr, const uint8_t *p, uint32_t n,
                       uint32_t busy_reg, uint32_t mask)
{
    if (!n)
        return true;
    uint32_t hi = (busy_reg + 0x800u) >> 12, lo = busy_reg - (hi << 12);
    const uint32_t prog[] = {
        INSN_SW_S1_S0,
        0x00440413u,                                             /* addi s0, s0, 4 */
        (hi << 12) | (9u << 7) | 0x37u,                          /* lui  s1, hi    */
        ((lo & 0xFFFu) << 20) | (9u << 15) | (2u << 12) | (9u << 7) | 0x03u, /* lw s1, lo(s1) */
        ((mask & 0x7FFu) << 20) | (9u << 15) | (7u << 12) | (9u << 7) | 0x13u, /* andi s1, s1, mask */
        0xFE049AE3u,                                             /* bnez s1, -12   */
        INSN_EBREAK,
    };
    progbuf_op = 0;
    for (unsigned i = 0; i < sizeof prog / sizeof prog[0]; i++)
        if (rvdm_write(DM_PROGBUF0 + i, prog[i]) != DMI_OK)
            return false;
    if (!reg_write(REG_S0, addr, false))
        return false;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t w = p[4 * i] | (p[4 * i + 1] << 8) | (p[4 * i + 2] << 16) |
                     ((uint32_t)p[4 * i + 3] << 24);
        rvdm_write(DM_DATA0, w);
        if (!command(CMD_REG | CMD_WRITE | CMD_POSTEXEC | REG_S1))
            return false;
    }
    return true;
}
