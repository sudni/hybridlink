/*
 * WCH one-wire SDI/SWIO: access to the RISC-V debug module (DMI) of
 * CH32V003 (and related V2A parts), bit-banged on the probe SWDIO pin.
 * SPDX-License-Identifier: MIT
 */
#ifndef SDI_H
#define SDI_H

#include <stdbool.h>
#include <stdint.h>

/* DMI status codes match rvswd.h / RISC-V debug spec */
#define DMI_OP_NOP      0
#define DMI_OP_READ     1
#define DMI_OP_WRITE    2
#define DMI_OK          0
#define DMI_FAILED      2

void     sdi_init(void);
void     sdi_set_delay(uint32_t d);     /* unused for fixed LinkE timing */
void     sdi_set_pinout(uint8_t swap);  /* 0: data=SWDIO NRST=SWCLK; 1: swapped */
uint8_t  sdi_pinout(void);
void     sdi_wake(void);                /* assert NRST + SWIO bus reset (NRST held) */
void     sdi_wake_run(void);            /* release NRST + fast SWIO reset (race remux) */
void     sdi_leave_reset(void);         /* ensure NRST released */
void     sdi_set_preset(uint8_t i);     /* timing table entry for attach sweep */
uint8_t  sdi_preset_count(void);
uint8_t  sdi_preset(void);
uint32_t sdi_calib_cyc(void);           /* DWT cycles for a 54-cycle RAM wait */
/* bit0..2 line ok; bit3=swap; bits15:8 preset; bits31:16 calib_cyc */
uint32_t sdi_line_probe(void);
uint8_t  sdi_read(uint8_t addr, uint32_t *val);
uint8_t  sdi_write(uint8_t addr, uint32_t val);

#endif
