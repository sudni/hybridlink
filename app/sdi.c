/*
 * WCH one-wire SDI/SWIO for CH32V003.
 *
 * Analyzer (20 MHz) LinkE vs Nano:
 *   LinkE: 1≈250 ns, 0≈850 ns, high≈350–400 ns, CAP → 0x00010403.
 *   2.45:  attach OK (hi≈450, 0≈700); regs false Parity (sample).
 *   2.47:  call wait-at-start → no 0-lows, hi≈950, CAP 0.
 *
 * SPDX-License-Identifier: MIT
 */
#include "sdi.h"
#include "led.h"
#include "swd_io.h"
#include "stm32f1.h"
#include "hal.h"
#include "board.h"

#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004u)
#define DEMCR       (*(volatile uint32_t *)0xE000EDFCu)

#ifndef SDI_PORT
#define SDI_PORT    SWDIO_PORT
#define SDI_PIN     SWDIO_PIN
#endif

#ifndef SDI_NRST_PORT
#define SDI_NRST_PORT   NRST_PORT
#define SDI_NRST_PIN    NRST_PIN
#endif

#define RAMFN   __attribute__((section(".ramfunc"), noinline, used))
#define RETRIES 6
#define PRESETS 4

typedef struct {
    uint16_t d1_l, d1_h, d0_l, d0_h;
    uint16_t rd_l, rd_sample, rd_bit, stop_h;
} sdi_timing_t;

/*
 * 72 MHz DWT. Compared to Ishu1519/esp32s3-ch32-programmer (CNLohr t1):
 *   ESP32-S3: t1≈200 ns; write 1=1×t1/1×t1, 0=4×t1/1×t1;
 *   read: low 1×t1, release, sample after +2×t1 → sample @ 3×t1≈600 ns.
 *   LinkE wire: ≈250/850/350. Target 0-hold ≈800 ns.
 *
 * 2.49 sampled @52 (~720 ns) — too close to end of 0-window on Dupont.
 * Preset0 = ESP/CNLohr; 1–3 = LinkE-ish with ESP sample point.
 */
static const sdi_timing_t presets[PRESETS] = {
    /* d1_l d1_h d0_l d0_h  rd_l rd_s rd_bit stop */
    { 15, 15, 60, 15, 15, 45, 90, 162 },  /* ESP t1=15cy≈208ns; sample@3t1 */
    { 18, 25, 72, 25, 18, 45, 100, 162 }, /* LinkE send + ESP sample */
    { 18, 28, 72, 28, 18, 42, 100, 162 }, /* sample a bit earlier */
    { 16, 22, 64, 22, 16, 48, 100, 180 }, /* slightly later sample */
};

static sdi_timing_t T;
static uint8_t preset_i;
static uint8_t pinout;
static uint32_t calib_cyc;

static uint32_t dio_port, nrst_port;
static unsigned dio_pin, nrst_pin;

static volatile uint32_t *sdi_bsrr;
static volatile uint32_t *sdi_idr;
static volatile uint32_t *sdi_cr;
static uint32_t sdi_pin_set, sdi_pin_clr, sdi_idr_mask, sdi_cr_shift;

#define WAIT_UNTIL(dl) \
    do { while ((int32_t)(DWT_CYCCNT - (dl)) < 0) { } } while (0)

RAMFN static void cr_out_pp(void)
{
    uint32_t v = *sdi_cr;
    v &= ~(0xFu << sdi_cr_shift);
    v |= (GPIO_OUT_PP_50 << sdi_cr_shift);
    *sdi_cr = v;
}

RAMFN static void cr_in_float(void)
{
    uint32_t v = *sdi_cr;
    v &= ~(0xFu << sdi_cr_shift);
    v |= (GPIO_IN_FLOAT << sdi_cr_shift);
    *sdi_cr = v;
}

RAMFN static void cr_in_pu(void)
{
    *sdi_bsrr = sdi_pin_set;
    uint32_t v = *sdi_cr;
    v &= ~(0xFu << sdi_cr_shift);
    v |= (GPIO_IN_PULL << sdi_cr_shift);
    *sdi_cr = v;
}

/*
 * Phase-locked send (2.45 style): fall, wait lo, rise, t0+=period, wait.
 * Loop OH after the trail WAIT shortens the next low (wire 1≈150, 0≈700 on
 * 2.45) — still enough for unlock/CAP. 2.47's bit_send_at() call made every
 * bit miss its deadline → no 0-lows at all.
 *
 * BSRR locals keep fall/rise tight; d0_l is padded so wire 0s stay ≥750 ns.
 */
#define BIT_SEND_AT(t0, one, bsrr, clr, set, d1l, d1h, d0l, d0h) do { \
    uint32_t _lo = (one) ? (d1l) : (d0l);                             \
    uint32_t _hi = (one) ? (d1h) : (d0h);                             \
    *(bsrr) = (clr);                                                  \
    WAIT_UNTIL((t0) + _lo);                                           \
    *(bsrr) = (set);                                                  \
    (t0) += _lo + _hi;                                                \
    WAIT_UNTIL(t0);                                                   \
} while (0)

RAMFN static uint32_t xfer_read(uint8_t addr)
{
    uint32_t dat = 0;
    uint8_t a = (uint8_t)(addr << 1);
    uint32_t t0, dl;
    int i;
    volatile uint32_t *bsrr = sdi_bsrr;
    volatile uint32_t *idr = sdi_idr;
    volatile uint32_t *cr = sdi_cr;
    uint32_t clr = sdi_pin_clr, set = sdi_pin_set, mask = sdi_idr_mask;
    uint32_t sh = sdi_cr_shift;
    uint32_t d1l = T.d1_l, d1h = T.d1_h, d0l = T.d0_l, d0h = T.d0_h;
    uint32_t rd_l = T.rd_l, rd_s = T.rd_sample, rd_b = T.rd_bit;

    *bsrr = set;
    cr_out_pp();
    __disable_irq();

    t0 = DWT_CYCCNT;
    BIT_SEND_AT(t0, 1, bsrr, clr, set, d1l, d1h, d0l, d0h);
    for (i = 0; i < 7; i++) {
        BIT_SEND_AT(t0, a & 0x80u, bsrr, clr, set, d1l, d1h, d0l, d0h);
        a <<= 1;
    }
    BIT_SEND_AT(t0, 0, bsrr, clr, set, d1l, d1h, d0l, d0h); /* R */

    /*
     * Read bits: wait for bit-start then fall (OH in high phase).
     * ESP/CNLohr: sample at 3×t1 after fall (~600 ns), still inside 0-hold.
     */
    for (i = 0; i < 32; i++) {
        uint32_t v;
        dat <<= 1;
        WAIT_UNTIL(t0);
        *bsrr = clr;
        WAIT_UNTIL(t0 + rd_l);
        v = *cr;
        v &= ~(0xFu << sh);
        v |= (GPIO_IN_FLOAT << sh);
        *cr = v;
        WAIT_UNTIL(t0 + rd_s);
        if (*idr & mask)
            dat++;
        *bsrr = set;
        v = *cr;
        v &= ~(0xFu << sh);
        v |= (GPIO_OUT_PP_50 << sh);
        *cr = v;
        t0 += rd_b;
    }

    WAIT_UNTIL(t0);
    dl = DWT_CYCCNT + T.stop_h;
    cr_in_float();
    WAIT_UNTIL(dl);
    __enable_irq();
    *bsrr = set;
    cr_out_pp();
    return dat;
}

RAMFN static void xfer_write(uint8_t addr, uint32_t val)
{
    uint8_t a = (uint8_t)((addr << 1) | 1u);
    uint32_t t0, dl;
    int i;
    volatile uint32_t *bsrr = sdi_bsrr;
    uint32_t clr = sdi_pin_clr, set = sdi_pin_set;
    uint32_t d1l = T.d1_l, d1h = T.d1_h, d0l = T.d0_l, d0h = T.d0_h;

    *bsrr = set;
    cr_out_pp();
    __disable_irq();

    t0 = DWT_CYCCNT;
    BIT_SEND_AT(t0, 1, bsrr, clr, set, d1l, d1h, d0l, d0h);
    for (i = 0; i < 7; i++) {
        BIT_SEND_AT(t0, a & 0x80u, bsrr, clr, set, d1l, d1h, d0l, d0h);
        a <<= 1;
    }
    BIT_SEND_AT(t0, 1, bsrr, clr, set, d1l, d1h, d0l, d0h);
    for (i = 0; i < 32; i++) {
        BIT_SEND_AT(t0, val & 0x80000000u, bsrr, clr, set, d1l, d1h, d0l, d0h);
        val <<= 1;
    }

    dl = DWT_CYCCNT + T.stop_h;
    cr_in_float();
    WAIT_UNTIL(dl);
    __enable_irq();
    *bsrr = set;
    cr_out_pp();
}

void sdi_set_delay(uint32_t d)
{
    (void)d;
}

void sdi_set_preset(uint8_t i)
{
    if (i >= PRESETS)
        i = 0;
    preset_i = i;
    T = presets[i];
}

uint8_t sdi_preset_count(void)
{
    return PRESETS;
}

uint8_t sdi_preset(void)
{
    return preset_i;
}

uint32_t sdi_calib_cyc(void)
{
    return calib_cyc;
}

uint8_t sdi_pinout(void)
{
    return pinout;
}

static void bind_pins(uint8_t swap)
{
    pinout = swap ? 1u : 0u;
    if (!swap) {
        dio_port = SDI_PORT;
        dio_pin = SDI_PIN;
        nrst_port = SDI_NRST_PORT;
        nrst_pin = SDI_NRST_PIN;
    } else {
        dio_port = SWCLK_PORT;
        dio_pin = SWCLK_PIN;
        nrst_port = SWDIO_PORT;
        nrst_pin = SWDIO_PIN;
    }

    sdi_bsrr = &GPIO_BSRR(dio_port);
    sdi_idr = &GPIO_IDR(dio_port);
    sdi_cr = &SWD_CR(dio_port, dio_pin);
    sdi_pin_set = 1u << dio_pin;
    sdi_pin_clr = 1u << (dio_pin + 16);
    sdi_idr_mask = 1u << dio_pin;
    sdi_cr_shift = SWD_SHIFT(dio_pin);

    GPIO_BSRR(nrst_port) = 1u << nrst_pin;
    gpio_mode(nrst_port, nrst_pin, GPIO_OUT_OD_10);
    cr_in_pu();
}

void sdi_set_pinout(uint8_t swap)
{
    bind_pins(swap);
}

void sdi_init(void)
{
    DEMCR |= 1u << 24;
    DWT_CTRL |= 1u;
    sdi_set_preset(0);
    bind_pins(0);
    {
        cr_out_pp();
        *sdi_bsrr = sdi_pin_set;
        __disable_irq();
        uint32_t t0 = DWT_CYCCNT;
        *sdi_bsrr = sdi_pin_clr;
        WAIT_UNTIL(t0 + presets[0].d1_l);
        uint32_t t1 = DWT_CYCCNT;
        *sdi_bsrr = sdi_pin_set;
        __enable_irq();
        calib_cyc = t1 - t0;
    }
}

static void sdi_nrst(bool assert)
{
    if (assert)
        GPIO_BRR(nrst_port) = 1u << nrst_pin;
    else
        GPIO_BSRR(nrst_port) = 1u << nrst_pin;
}

RAMFN static void swio_preamble(void)
{
    int i;
    *sdi_bsrr = sdi_pin_set;
    cr_out_pp();
    __disable_irq();
    for (i = 0; i < 32; i++) {
        uint32_t dl = DWT_CYCCNT + 65u;
        *sdi_bsrr = sdi_pin_clr;
        WAIT_UNTIL(dl);
        dl = DWT_CYCCNT + 25u;
        *sdi_bsrr = sdi_pin_set;
        WAIT_UNTIL(dl);
    }
    {
        uint32_t dl = DWT_CYCCNT + T.stop_h;
        cr_in_float();
        WAIT_UNTIL(dl);
    }
    __enable_irq();
}

void sdi_wake(void)
{
    sdi_nrst(true);
    hal_delay_ms(30);
    cr_out_pp();
    *sdi_bsrr = sdi_pin_clr;
    hal_delay_ms(2);
    *sdi_bsrr = sdi_pin_set;
    {
        uint32_t dl = DWT_CYCCNT + T.stop_h;
        cr_in_float();
        WAIT_UNTIL(dl);
    }
    swio_preamble();
}

void sdi_wake_run(void)
{
    sdi_nrst(false);
    hal_delay_us(50);
    cr_out_pp();
    *sdi_bsrr = sdi_pin_clr;
    hal_delay_ms(2);
    *sdi_bsrr = sdi_pin_set;
    {
        uint32_t dl = DWT_CYCCNT + T.stop_h;
        cr_in_float();
        WAIT_UNTIL(dl);
    }
    swio_preamble();
}

void sdi_leave_reset(void)
{
    sdi_nrst(false);
}

uint32_t sdi_line_probe(void)
{
    uint32_t r = 0;
    uint32_t t0;
    cr_out_pp();
    *sdi_bsrr = sdi_pin_clr;
    t0 = DWT_CYCCNT;
    while ((DWT_CYCCNT - t0) < 1000u) { }
    if (!(*sdi_idr & sdi_idr_mask))
        r |= 1u;
    cr_in_float();
    *sdi_bsrr = sdi_pin_set;
    t0 = DWT_CYCCNT;
    while ((DWT_CYCCNT - t0) < 3000u) { }
    if (*sdi_idr & sdi_idr_mask)
        r |= 2u;
    cr_out_pp();
    *sdi_bsrr = sdi_pin_clr;
    t0 = DWT_CYCCNT;
    while ((DWT_CYCCNT - t0) < 500u) { }
    cr_in_float();
    t0 = DWT_CYCCNT;
    while ((DWT_CYCCNT - t0) < 10000u) { }
    if (*sdi_idr & sdi_idr_mask)
        r |= 4u;
    if (pinout)
        r |= 8u;
    cr_in_pu();
    r |= ((uint32_t)preset_i << 8);
    r |= (calib_cyc & 0xffffu) << 16;
    return r;
}

static uint8_t dmi_done(uint8_t st)
{
    led_transfer(st == DMI_OK);
    return st;
}

uint8_t sdi_write(uint8_t addr, uint32_t val)
{
    xfer_write(addr, val);
    return dmi_done(DMI_OK);
}

uint8_t sdi_read(uint8_t addr, uint32_t *val)
{
    uint32_t v = 0xffffffffu;
    for (int t = 0; t < RETRIES; t++) {
        v = xfer_read(addr);
        if (v != 0xffffffffu && v != 0u)
            break;
    }
    *val = v;
    return dmi_done(DMI_OK);
}
