/*
 * NanoDAP (MuseLab / STM32F103C8/CB) @ 8 MHz HSE.
 *
 * Target SWD matches ST-Link/V2 and MuseLab nanoDAP V2.x (PB13/PB14 via 100R):
 *   PB13  SWCLK (target, via 100R)
 *   PB14  SWDIO (drive + sense on the same pin; no PB12 tap)
 *   PB0   target NRST (open drain, external pull-up)
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

#define NRST_PORT           GPIOB_BASE
#define NRST_PIN            0

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
