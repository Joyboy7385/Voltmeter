/*---------------------------------------------------------------------------------------------------------*/
/*  SFR_MACRO_SDCC.H - SDCC-Compatible SFR Macros for MS51                                               */
/*---------------------------------------------------------------------------------------------------------*/
/*
 * This header provides macro definitions for SFR bit operations,
 * converted from Keil C51 format to SDCC format.
 *
 * In Keil: set_XXX_YYY macros expand to bit operations
 * In SDCC: We use direct bit assignments instead
 *
 * Date: 2025-11-19
 */

#ifndef __SFR_MACRO_SDCC_H__
#define __SFR_MACRO_SDCC_H__

#include "ms51_sdcc.h"

/*===================================================================================
 * BIT OPERATIONS - SDCC uses direct assignments instead of macros
 *===================================================================================*/

/* NOTE: In SDCC, we don't need these macros because we can directly access bits.
 * However, to maintain compatibility with the original Keil code, we define them
 * as simple assignments.
 */

/*------- ADCCON0 Bit Operations (Critical for voltmeter) -------*/
#define set_ADCS                ADCS = 1
#define clr_ADCS                ADCS = 0
#define set_ADCF                ADCF = 1
#define clr_ADCF                ADCF = 0

/*------- CHPCON Bit Operations (IAP/EEPROM access) -------*/
/* Note: CHPCON is not bit-addressable on all 8051 variants,
 * but these operations work on MS51 */
#define set_CHPCON_IAPEN        do { CHPCON |= 0x01; } while(0)
#define clr_CHPCON_IAPEN        do { CHPCON &= ~0x01; } while(0)

/*------- IAPTRG Bit Operations (IAP trigger) -------*/
#define set_IAPTRG_IAPGO        do { IAPTRG |= 0x01; } while(0)
#define clr_IAPTRG_IAPGO        do { IAPTRG &= ~0x01; } while(0)

/*------- WDCON Bit Operations (Watchdog control) -------*/
/* Note: These require checking MS51 datasheet for exact bit positions */
#define set_WDCON_WDTR          do { WDCON |= 0x40; } while(0)  /* Bit 6 */
#define clr_WDCON_WDTR          do { WDCON &= ~0x40; } while(0)
#define set_WDCON_WDCLR         do { WDCON |= 0x20; } while(0)  /* Bit 5 */
#define clr_WDCON_WDCLR         do { WDCON &= ~0x20; } while(0)

/*------- TCON Bit Operations (Timer control) -------*/
#define set_TCON_TR0            TR0 = 1
#define clr_TCON_TR0            TR0 = 0
#define set_TCON_TF0            TF0 = 1
#define clr_TCON_TF0            TF0 = 0
#define set_TCON_TR1            TR1 = 1
#define clr_TCON_TR1            TR1 = 0
#define set_TCON_TF1            TF1 = 1
#define clr_TCON_TF1            TF1 = 0

/*------- IE Bit Operations (Interrupt enable) -------*/
#define set_IE_EA               EA = 1
#define clr_IE_EA               EA = 0
#define set_IE_EADC             EADC = 1
#define clr_IE_EADC             EADC = 0

/*------- SFRS Operations (SFR Page Select) -------*/
#define clr_SFRS_SFRPAGE        do { SFRS &= ~0x01; } while(0)
#define set_SFRS_SFRPAGE        do { SFRS |= 0x01; } while(0)

/*===================================================================================
 * PIN MODE CONFIGURATION MACROS
 *===================================================================================*/

/* Pin Mode Configuration:
 * MS51 uses two registers (PxM1, PxM2) to configure pin modes:
 *
 * M1  M2  Mode
 * --  --  ----
 * 0   0   Quasi-bidirectional (weak pull-up)
 * 0   1   Push-pull output
 * 1   0   Input only (high impedance)
 * 1   1   Open-drain output
 */

/*------- Port 0 Pin Modes -------*/
#define P00_PUSHPULL_MODE       do { SFRS &= ~0x01; P0M1 &= ~0x01; P0M2 |= 0x01; } while(0)
#define P01_PUSHPULL_MODE       do { SFRS &= ~0x01; P0M1 &= ~0x02; P0M2 |= 0x02; } while(0)
#define P02_PUSHPULL_MODE       do { SFRS &= ~0x01; P0M1 &= ~0x04; P0M2 |= 0x04; } while(0)
#define P03_PUSHPULL_MODE       do { SFRS &= ~0x01; P0M1 &= ~0x08; P0M2 |= 0x08; } while(0)
#define P04_PUSHPULL_MODE       do { SFRS &= ~0x01; P0M1 &= ~0x10; P0M2 |= 0x10; } while(0)
#define P05_PUSHPULL_MODE       do { SFRS &= ~0x01; P0M1 &= ~0x20; P0M2 |= 0x20; } while(0)
#define P06_PUSHPULL_MODE       do { SFRS &= ~0x01; P0M1 &= ~0x40; P0M2 |= 0x40; } while(0)
#define P07_PUSHPULL_MODE       do { SFRS &= ~0x01; P0M1 &= ~0x80; P0M2 |= 0x80; } while(0)

#define P00_QUASI_MODE          do { SFRS &= ~0x01; P0M1 &= ~0x01; P0M2 &= ~0x01; } while(0)
#define P01_QUASI_MODE          do { SFRS &= ~0x01; P0M1 &= ~0x02; P0M2 &= ~0x02; } while(0)
#define P02_QUASI_MODE          do { SFRS &= ~0x01; P0M1 &= ~0x04; P0M2 &= ~0x04; } while(0)
#define P03_QUASI_MODE          do { SFRS &= ~0x01; P0M1 &= ~0x08; P0M2 &= ~0x08; } while(0)
#define P04_QUASI_MODE          do { SFRS &= ~0x01; P0M1 &= ~0x10; P0M2 &= ~0x10; } while(0)
#define P05_QUASI_MODE          do { SFRS &= ~0x01; P0M1 &= ~0x20; P0M2 &= ~0x20; } while(0)
#define P06_QUASI_MODE          do { SFRS &= ~0x01; P0M1 &= ~0x40; P0M2 &= ~0x40; } while(0)
#define P07_QUASI_MODE          do { SFRS &= ~0x01; P0M1 &= ~0x80; P0M2 &= ~0x80; } while(0)

/*------- Port 1 Pin Modes -------*/
#define P10_PUSHPULL_MODE       do { SFRS &= ~0x01; P1M1 &= ~0x01; P1M2 |= 0x01; } while(0)
#define P11_PUSHPULL_MODE       do { SFRS &= ~0x01; P1M1 &= ~0x02; P1M2 |= 0x02; } while(0)
#define P12_PUSHPULL_MODE       do { SFRS &= ~0x01; P1M1 &= ~0x04; P1M2 |= 0x04; } while(0)
#define P13_PUSHPULL_MODE       do { SFRS &= ~0x01; P1M1 &= ~0x08; P1M2 |= 0x08; } while(0)
#define P14_PUSHPULL_MODE       do { SFRS &= ~0x01; P1M1 &= ~0x10; P1M2 |= 0x10; } while(0)
#define P15_PUSHPULL_MODE       do { SFRS &= ~0x01; P1M1 &= ~0x20; P1M2 |= 0x20; } while(0)
#define P16_PUSHPULL_MODE       do { SFRS &= ~0x01; P1M1 &= ~0x40; P1M2 |= 0x40; } while(0)
#define P17_PUSHPULL_MODE       do { SFRS &= ~0x01; P1M1 &= ~0x80; P1M2 |= 0x80; } while(0)

#define P10_QUASI_MODE          do { SFRS &= ~0x01; P1M1 &= ~0x01; P1M2 &= ~0x01; } while(0)
#define P11_QUASI_MODE          do { SFRS &= ~0x01; P1M1 &= ~0x02; P1M2 &= ~0x02; } while(0)
#define P12_QUASI_MODE          do { SFRS &= ~0x01; P1M1 &= ~0x04; P1M2 &= ~0x04; } while(0)
#define P13_QUASI_MODE          do { SFRS &= ~0x01; P1M1 &= ~0x08; P1M2 &= ~0x08; } while(0)
#define P14_QUASI_MODE          do { SFRS &= ~0x01; P1M1 &= ~0x10; P1M2 &= ~0x10; } while(0)
#define P15_QUASI_MODE          do { SFRS &= ~0x01; P1M1 &= ~0x20; P1M2 &= ~0x20; } while(0)
#define P16_QUASI_MODE          do { SFRS &= ~0x01; P1M1 &= ~0x40; P1M2 &= ~0x40; } while(0)
#define P17_QUASI_MODE          do { SFRS &= ~0x01; P1M1 &= ~0x80; P1M2 &= ~0x80; } while(0)

#define P17_Input_Mode          do { P1M1 |= 0x80; P1M2 &= ~0x80; } while(0)

/*------- Port 3 Pin Modes -------*/
#define P30_Input_Mode          do { P3M1 |= 0x01; P3M2 &= ~0x01; } while(0)
#define P30_QUASI_MODE          do { P3M1 &= ~0x01; P3M2 &= ~0x01; } while(0)

/*===================================================================================
 * HELPER MACROS FOR TIMER0 (Used in delay function)
 *===================================================================================*/

/* Timer0 Mode 1: 16-bit timer/counter */
#define ENABLE_TIMER0_MODE1     do { TMOD = (TMOD & 0xF0) | 0x01; } while(0)

/* Timer0 clock source = Fsys/12 */
#define TIMER0_FSYS_DIV12       do { CKCON &= ~0x08; } while(0)

/*===================================================================================
 * ADC RELATED MACROS
 *===================================================================================*/

/* Set bit in AINDIDS to disable digital input buffer (for ADC pins) */
#define SET_BIT0                0x01
#define SET_BIT1                0x02
#define SET_BIT2                0x04
#define SET_BIT3                0x08
#define SET_BIT4                0x10
#define SET_BIT5                0x20
#define SET_BIT6                0x40
#define SET_BIT7                0x80

#endif /* __SFR_MACRO_SDCC_H__ */
