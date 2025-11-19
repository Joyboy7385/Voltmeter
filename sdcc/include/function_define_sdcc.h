/*---------------------------------------------------------------------------------------------------------*/
/*  FUNCTION_DEFINE_SDCC.H - Helper Macros and Type Definitions for SDCC                                 */
/*---------------------------------------------------------------------------------------------------------*/
/*
 * This header provides helper macros for byte manipulation,
 * converted from Keil C51 format to SDCC format.
 *
 * Date: 2025-11-19
 */

#ifndef __FUNCTION_DEFINE_SDCC_H__
#define __FUNCTION_DEFINE_SDCC_H__

/*===================================================================================
 * TYPE DEFINITIONS
 *===================================================================================*/

/* Basic type definitions (compatible with stdint.h) */
typedef unsigned char       UINT8;
typedef unsigned int        UINT16;
typedef unsigned long       UINT32;
typedef unsigned char       uint8_t;
typedef unsigned int        uint16_t;
typedef unsigned long       uint32_t;

typedef signed char         INT8;
typedef signed int          INT16;
typedef signed long         INT32;
typedef signed char         int8_t;
typedef signed int          int16_t;
typedef signed long         int32_t;

/*===================================================================================
 * BYTE MANIPULATION MACROS
 *===================================================================================*/

/* Extract high byte from 16-bit value */
#define HIBYTE(v1)              ((UINT8)((v1) >> 8))

/* Extract low byte from 16-bit value */
#define LOBYTE(v1)              ((UINT8)((v1) & 0xFF))

/* Combine two bytes into 16-bit value */
#define MAKEWORD(v1,v2)         ((((UINT16)(v1)) << 8) + (UINT16)(v2))

/* Combine four bytes into 32-bit value */
#define MAKELONG(v1,v2,v3,v4)   ((UINT32)((v1 << 24) + (v2 << 16) + (v3 << 8) + v4))

/* Extract upper 16 bits from 32-bit value */
#define YBYTE1(v1)              ((UINT16)((v1) >> 16))

/* Extract lower 16 bits from 32-bit value */
#define YBYTE0(v1)              ((UINT16)((v1) & 0xFFFF))

/* Extract individual bytes from 32-bit value */
#define TBYTE3(v1)              ((UINT8)((v1) >> 24))
#define TBYTE2(v1)              ((UINT8)((v1) >> 16))
#define TBYTE1(v1)              ((UINT8)((v1) >> 8))
#define TBYTE0(v1)              ((UINT8)((v1) & 0xFF))

/*===================================================================================
 * BIT MASKS
 *===================================================================================*/

/* Set bit masks */
#define SET_BIT0                0x01
#define SET_BIT1                0x02
#define SET_BIT2                0x04
#define SET_BIT3                0x08
#define SET_BIT4                0x10
#define SET_BIT5                0x20
#define SET_BIT6                0x40
#define SET_BIT7                0x80

/* Clear bit masks */
#define CLR_BIT0                0xFE
#define CLR_BIT1                0xFD
#define CLR_BIT2                0xFB
#define CLR_BIT3                0xF7
#define CLR_BIT4                0xEF
#define CLR_BIT5                0xDF
#define CLR_BIT6                0xBF
#define CLR_BIT7                0x7F

/*===================================================================================
 * COMMON CONSTANTS
 *===================================================================================*/

#define Disable                 0
#define Enable                  1

#define TRUE                    1
#define FALSE                   0

#define PASS                    0
#define FAIL                    1

/*===================================================================================
 * SDCC-SPECIFIC DEFINITIONS
 *===================================================================================*/

/* SDCC uses __bit instead of bit for bit variables */
#ifndef bit
#define bit                     __bit
#endif

/* NOP instruction (SDCC version) */
#define _nop_()                 __asm nop __endasm

#endif /* __FUNCTION_DEFINE_SDCC_H__ */
