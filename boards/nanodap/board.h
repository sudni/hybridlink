/*
 * NanoDAP (MuseLab / STM32F103C8/CB) @ 8 MHz HSE.
 *
 * Target SWD matches MuseLab nanoDAP V2.x (validated with ST-Link + CMSIS-DAP
 * against an RP2040). nanoDAP V2.1 sch: 100 Ω only on TCK (R10), TMS direct:
 *   PB13  SWCLK / TCK  (via R10 100 Ω) — SDI target NRST on sticks
 *         without a broken-out PB0 (QSZNTEC 5-pin: SWCLK→CH32 NRST)
 *   PB14  SWDIO / TMS  (direct) — SDI here; PD1 + 1 kΩ to 3V3
 *   PB0   target NRST for DAP/ST (open drain); unused on 5-pin sticks
 *   PA9   LED_RUN  (red, active high) — DAPLink HIC; some V2.x use PA8
 *   PB6   LED_CON  (blue, active high)
 *   PA15  USB_ENUM (PNP Q1 base; low = D+ pull-up) — some V2.x use PB15
 *   PA2   VCP TX  (USART2)        PA3  VCP RX  (USART2)
 *
 * No SWO input and no target voltage sense.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef BOARD_H
#define BOARD_H

#define BOARD_NAME          "NanoDAP"
#define BOARD_HSE_HZ        8000000u

#define SWCLK_PORT          GPIOB_BASE
#define SWCLK_PIN           13
#define SWDIO_PORT          GPIOB_BASE
#define SWDIO_PIN           14
#define SWDIO_IN_PORT       GPIOB_BASE
#define SWDIO_IN_PIN        14

/* One-wire SDI on TMS/SWDIO (no series R on V2.1) */
#define SDI_PORT            SWDIO_PORT
#define SDI_PIN             SWDIO_PIN
#define SDI_IN_PORT         SWDIO_IN_PORT
#define SDI_IN_PIN          SWDIO_IN_PIN

#define NRST_PORT           GPIOB_BASE
#define NRST_PIN            0

/* One-wire SDI: drive target NRST on SWCLK (header pin exists; PB0 does not) */
#define SDI_NRST_PORT       SWCLK_PORT
#define SDI_NRST_PIN        SWCLK_PIN

#define LED_PORT            GPIOA_BASE
#define LED_PIN             9               /* RUN (red) */
#define LED_CON_PORT        GPIOB_BASE
#define LED_CON_PIN         6               /* CON (blue) */
#define BOARD_LED_ACTIVE_HIGH 1

#define USB_RENUM_PORT      GPIOA_BASE
#define USB_RENUM_PIN       15
#define USB_RENUM_ACTIVE_LOW 1

#define VCP_TX_PORT         GPIOA_BASE
#define VCP_TX_PIN          2
#define VCP_RX_PORT         GPIOA_BASE
#define VCP_RX_PIN          3

#endif
