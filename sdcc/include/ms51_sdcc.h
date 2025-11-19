/*---------------------------------------------------------------------------------------------------------*/
/*  MS51_SDCC.H - SDCC-Compatible Header for Nuvoton MS51FB9AE / MS51XB9AE / MS51XB9BE                  */
/*---------------------------------------------------------------------------------------------------------*/
/*
 * This header provides SFR (Special Function Register) definitions for the
 * Nuvoton MS51 series microcontrollers, compatible with SDCC compiler.
 *
 * Converted from Keil C51 format to SDCC format.
 * Date: 2025-11-19
 */

#ifndef __MS51_SDCC_H__
#define __MS51_SDCC_H__

#include <8051.h>  /* SDCC standard 8051 definitions */

/*===================================================================================
 * SFR REGISTER DEFINITIONS
 * Format: __sfr __at (address) REGISTER_NAME;
 *===================================================================================*/

/* Standard 8051 SFRs (already in 8051.h, but listed for reference) */
/* P0, SP, DPL, DPH, PCON, TCON, TMOD, TL0, TL1, TH0, TH1, P1, SCON, SBUF, */
/* IE, P2, IP, PSW, ACC, B are already defined in <8051.h> */

/* MS51-Specific SFRs */
__sfr __at (0x84) RCTRIM0;
__sfr __at (0x85) RCTRIM1;
__sfr __at (0x86) RWK;

__sfr __at (0x8E) CKCON;      /* Clock Control (also called AUXR0 in some docs) */
__sfr __at (0x8E) AUXR0;      /* Alias for CKCON - controls P2.0 reset function */
__sfr __at (0x8F) WKCON;

__sfr __at (0x91) SFRS;       /* SFR Page Select (TA protected) */
__sfr __at (0x92) CAPCON0;
__sfr __at (0x93) CAPCON1;
__sfr __at (0x94) CAPCON2;
__sfr __at (0x95) CKDIV;
__sfr __at (0x96) CKSWT;      /* Clock Switch (TA protected) */
__sfr __at (0x97) CKEN;       /* Clock Enable (TA protected) */

__sfr __at (0x99) SBUF_1;
__sfr __at (0x9B) EIE;        /* Extended Interrupt Enable */
__sfr __at (0x9C) EIE1;       /* Extended Interrupt Enable 1 */
__sfr __at (0x9F) CHPCON;     /* Chip Control (TA protected) */

__sfr __at (0xA0) P2;         /* Port 2 */
__sfr __at (0xA2) AUXR1;
__sfr __at (0xA3) BODCON0;    /* Brown-Out Detect Control 0 (TA protected) */
__sfr __at (0xA4) IAPTRG;     /* IAP Trigger (TA protected) */
__sfr __at (0xA5) IAPUEN;     /* IAP Update Enable (TA protected) */
__sfr __at (0xA6) IAPAL;      /* IAP Address Low */
__sfr __at (0xA7) IAPAH;      /* IAP Address High */

__sfr __at (0xA9) SADDR;
__sfr __at (0xAA) WDCON;      /* Watchdog Control (TA protected) */
__sfr __at (0xAB) BODCON1;    /* Brown-Out Detect Control 1 (TA protected) */
__sfr __at (0xAC) P3M1;       /* Port 3 Mode 1 */
__sfr __at (0xAD) P3M2;       /* Port 3 Mode 2 */
__sfr __at (0xAE) IAPFD;      /* IAP Flash Data */
__sfr __at (0xAF) IAPCN;      /* IAP Control */

__sfr __at (0xB0) P3;         /* Port 3 */
__sfr __at (0xB1) P0M1;       /* Port 0 Mode 1 */
__sfr __at (0xB2) P0M2;       /* Port 0 Mode 2 */
__sfr __at (0xB3) P1M1;       /* Port 1 Mode 1 */
__sfr __at (0xB4) P1M2;       /* Port 1 Mode 2 */
__sfr __at (0xB5) P2S;        /* Port 2 Slew Rate */
__sfr __at (0xB7) IPH;        /* Interrupt Priority High */

__sfr __at (0xB9) SADEN;
__sfr __at (0xBA) SADEN_1;
__sfr __at (0xBB) SADDR_1;
__sfr __at (0xBC) I2DAT;      /* I2C Data */
__sfr __at (0xBD) I2STAT;     /* I2C Status */
__sfr __at (0xBE) I2CLK;      /* I2C Clock */
__sfr __at (0xBF) I2TOC;      /* I2C Time-Out Control */

__sfr __at (0xC0) I2CON;      /* I2C Control */
__sfr __at (0xC1) I2ADDR;     /* I2C Address */
__sfr __at (0xC2) ADCRL;      /* ADC Result Low */
__sfr __at (0xC3) ADCRH;      /* ADC Result High */
__sfr __at (0xC4) T3CON;      /* Timer 3 Control */
__sfr __at (0xC5) RL3;        /* Timer 3 Reload Low */
__sfr __at (0xC6) RH3;        /* Timer 3 Reload High */
__sfr __at (0xC7) TA;         /* Timed Access */

__sfr __at (0xC8) T2CON;      /* Timer 2 Control */
__sfr __at (0xC9) T2MOD;      /* Timer 2 Mode */
__sfr __at (0xCA) RCMP2L;     /* Timer 2 Compare Low */
__sfr __at (0xCB) RCMP2H;     /* Timer 2 Compare High */
__sfr __at (0xCC) TL2;        /* Timer 2 Low */
__sfr __at (0xCD) TH2;        /* Timer 2 High */
__sfr __at (0xCE) ADCMPL;     /* ADC Compare Low */
__sfr __at (0xCF) ADCMPH;     /* ADC Compare High */

__sfr __at (0xD0) PSW;        /* Program Status Word */
__sfr __at (0xD1) PWMPH;
__sfr __at (0xD2) PWM0H;
__sfr __at (0xD3) PWM1H;
__sfr __at (0xD4) PWM2H;
__sfr __at (0xD5) PWM3H;
__sfr __at (0xD6) PNP;
__sfr __at (0xD7) FBD;

__sfr __at (0xD8) PWMCON0;    /* PWM Control 0 */
__sfr __at (0xD9) PWMPL;
__sfr __at (0xDA) PWM0L;
__sfr __at (0xDB) PWM1L;
__sfr __at (0xDC) PWM2L;
__sfr __at (0xDD) PWM3L;
__sfr __at (0xDE) PIOCON0;
__sfr __at (0xDF) PWMCON1;

__sfr __at (0xE0) ACC;        /* Accumulator */
__sfr __at (0xE1) ADCCON1;    /* ADC Control 1 */
__sfr __at (0xE2) ADCCON2;    /* ADC Control 2 */
__sfr __at (0xE3) ADCDLY;     /* ADC Delay */
__sfr __at (0xE4) C0L;
__sfr __at (0xE5) C0H;
__sfr __at (0xE6) C1L;
__sfr __at (0xE7) C1H;

__sfr __at (0xE8) ADCCON0;    /* ADC Control 0 */
__sfr __at (0xE9) PICON;      /* Pin Interrupt Control */
__sfr __at (0xEA) PINEN;      /* Pin Interrupt Negative Enable */
__sfr __at (0xEB) PIPEN;      /* Pin Interrupt Positive Enable */
__sfr __at (0xEC) PIF;        /* Pin Interrupt Flag */
__sfr __at (0xED) C2L;
__sfr __at (0xEE) C2H;
__sfr __at (0xEF) EIP;        /* Extended Interrupt Priority */

__sfr __at (0xF0) B;          /* B Register */
__sfr __at (0xF1) CAPCON3;
__sfr __at (0xF2) CAPCON4;
__sfr __at (0xF3) SPCR;       /* SPI Control */
__sfr __at (0xF4) SPSR;       /* SPI Status */
__sfr __at (0xF5) SPDR;       /* SPI Data */
__sfr __at (0xF6) AINDIDS;    /* ADC Input Disable */
__sfr __at (0xF7) EIPH;       /* Extended Interrupt Priority High */

__sfr __at (0xF8) SCON_1;     /* Serial Control 1 */
__sfr __at (0xF9) PDTEN;      /* Pin Detect Enable (TA protected) */
__sfr __at (0xFA) PDTCNT;     /* Pin Detect Counter (TA protected) */
__sfr __at (0xFB) PMEN;
__sfr __at (0xFC) PMD;
__sfr __at (0xFE) EIP1;       /* Extended Interrupt Priority 1 */
__sfr __at (0xFF) EIPH1;      /* Extended Interrupt Priority High 1 */

/*===================================================================================
 * BIT DEFINITIONS
 * Format: __sbit __at (address) BIT_NAME;
 * Address calculation: SFR_address*8 + bit_number (for bit-addressable SFRs)
 *===================================================================================*/

/* P0 bits (0x80) - P0 is bit-addressable */
__sbit __at (0x80) P00;
__sbit __at (0x80) MOSI;   /* Alternate function */
__sbit __at (0x81) P01;
__sbit __at (0x81) MISO;   /* Alternate function */
__sbit __at (0x82) P02;
__sbit __at (0x82) RXD_1;  /* Alternate function */
__sbit __at (0x83) P03;
__sbit __at (0x84) P04;
__sbit __at (0x84) STADC;  /* Alternate function */
__sbit __at (0x85) P05;
__sbit __at (0x86) P06;
__sbit __at (0x86) TXD;    /* Alternate function */
__sbit __at (0x87) P07;
__sbit __at (0x87) RXD;    /* Alternate function */

/* TCON bits (0x88) */
__sbit __at (0x88) IT0;
__sbit __at (0x89) IE0;
__sbit __at (0x8A) IT1;
__sbit __at (0x8B) IE1;
__sbit __at (0x8C) TR0;
__sbit __at (0x8D) TF0;
__sbit __at (0x8E) TR1;
__sbit __at (0x8F) TF1;

/* P1 bits (0x90) */
__sbit __at (0x90) P10;
__sbit __at (0x91) P11;
__sbit __at (0x92) P12;
__sbit __at (0x93) P13;
__sbit __at (0x93) SCL;    /* Alternate function */
__sbit __at (0x94) P14;
__sbit __at (0x94) SDA;    /* Alternate function */
__sbit __at (0x95) P15;
__sbit __at (0x96) P16;
__sbit __at (0x96) TXD_1;  /* Alternate function */
__sbit __at (0x97) P17;

/* SCON bits (0x98) */
__sbit __at (0x98) RI;
__sbit __at (0x99) TI;
__sbit __at (0x9A) RB8;
__sbit __at (0x9B) TB8;
__sbit __at (0x9C) REN;
__sbit __at (0x9D) SM2;
__sbit __at (0x9E) SM1;
__sbit __at (0x9F) SM0;
__sbit __at (0x9F) FE;     /* Alternate function */

/* P2 bits (0xA0) */
__sbit __at (0xA0) P20;

/* IE bits (0xA8) */
__sbit __at (0xA8) EX0;
__sbit __at (0xA9) ET0;
__sbit __at (0xAA) EX1;
__sbit __at (0xAB) ET1;
__sbit __at (0xAC) ES;
__sbit __at (0xAD) EBOD;
__sbit __at (0xAE) EADC;
__sbit __at (0xAF) EA;

/* P3 bits (0xB0) */
__sbit __at (0xB0) P30;

/* IP bits (0xB8) */
__sbit __at (0xB8) PX0;
__sbit __at (0xB9) PT0;
__sbit __at (0xBA) PX1;
__sbit __at (0xBB) PT1;
__sbit __at (0xBC) PS;
__sbit __at (0xBD) PBOD;
__sbit __at (0xBE) PADC;

/* I2CON bits (0xC0) */
__sbit __at (0xC0) I2CPX;
__sbit __at (0xC2) AA;
__sbit __at (0xC3) SI;
__sbit __at (0xC4) STO;
__sbit __at (0xC5) STA;
__sbit __at (0xC6) I2CEN;

/* T2CON bits (0xC8) */
__sbit __at (0xC8) CM_RL2;
__sbit __at (0xCA) TR2;
__sbit __at (0xCF) TF2;

/* PSW bits (0xD0) */
__sbit __at (0xD0) P;
__sbit __at (0xD2) OV;
__sbit __at (0xD3) RS0;
__sbit __at (0xD4) RS1;
__sbit __at (0xD5) F0;
__sbit __at (0xD6) AC;
__sbit __at (0xD7) CY;

/* PWMCON0 bits (0xD8) */
__sbit __at (0xDC) CLRPWM;
__sbit __at (0xDD) PWMF;
__sbit __at (0xDE) LOAD;
__sbit __at (0xDF) PWMRUN;

/* ADCCON0 bits (0xE8) - CRITICAL for voltmeter */
__sbit __at (0xE8) ADCHS0;   /* ADC Channel Select bit 0 */
__sbit __at (0xE9) ADCHS1;   /* ADC Channel Select bit 1 */
__sbit __at (0xEA) ADCHS2;   /* ADC Channel Select bit 2 */
__sbit __at (0xEB) ADCHS3;   /* ADC Channel Select bit 3 */
__sbit __at (0xEC) ETGSEL0;  /* External Trigger Select 0 */
__sbit __at (0xED) ETGSEL1;  /* External Trigger Select 1 */
__sbit __at (0xEE) ADCS;     /* ADC Start */
__sbit __at (0xEF) ADCF;     /* ADC Flag */

/* SCON_1 bits (0xF8) */
__sbit __at (0xF8) RI_1;
__sbit __at (0xF9) TI_1;
__sbit __at (0xFA) RB8_1;
__sbit __at (0xFB) TB8_1;
__sbit __at (0xFC) REN_1;
__sbit __at (0xFD) SM2_1;
__sbit __at (0xFE) SM1_1;
__sbit __at (0xFF) SM0_1;
__sbit __at (0xFF) FE_1;    /* Alternate function */

#endif /* __MS51_SDCC_H__ */
