/*---------------------------------------------------------------------------------------------------------*/
/*  DELAY_SDCC.C - Timer-based delay functions for SDCC                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/*
 * Timer0-based delay function ported from Keil C51 to SDCC.
 *
 * Original: Nuvoton MS51 delay library
 * Ported: 2025-11-19
 * Compiler: SDCC
 */

#include "ms51_sdcc.h"
#include "sfr_macro_sdcc.h"
#include "function_define_sdcc.h"

/**
 * @brief Timer0-based delay function
 * @param u32SYSCLK System clock frequency in Hz (e.g., 16000000 for 16MHz)
 * @param u16CNT Number of delay cycles to execute
 * @param u16DLYUnit Delay unit in microseconds (e.g., 1000 for 1ms)
 *
 * @note Timer0 operates in Mode 1 (16-bit timer) with clock = Fsys/12
 * @note Maximum u16DLYUnit is ~10000us to avoid overflow
 * @note This function blocks execution during the delay
 *
 * @example
 *   Timer0_Delay(16000000, 1, 1000);  // 1ms delay at 16MHz
 *   Timer0_Delay(16000000, 10, 100);  // 10 x 100us = 1ms delay at 16MHz
 */
void Timer0_Delay(unsigned long u32SYSCLK, unsigned int u16CNT, unsigned int u16DLYUnit)
{
    unsigned char TL0TMP, TH0TMP;
    unsigned int timer_reload;

    /* Configure Timer0:
     * - Mode 1: 16-bit timer
     * - Clock source: Fsys/12
     */
    TIMER0_FSYS_DIV12;              /* Set Timer0 clock = Fsys/12 */
    ENABLE_TIMER0_MODE1;            /* Set Timer0 to 16-bit mode */

    /* Calculate timer reload value:
     * Timer counts from reload value to 65535 (overflow)
     * Timer clock = Fsys/12
     * For delay of u16DLYUnit microseconds:
     *   ticks = (Fsys / 1,000,000) * u16DLYUnit / 12
     *   reload = 65536 - ticks
     */
    timer_reload = 65535 - ((u32SYSCLK / 1000000UL) * u16DLYUnit / 12);

    TL0TMP = LOBYTE(timer_reload);
    TH0TMP = HIBYTE(timer_reload);

    /* Execute delay loop */
    while (u16CNT != 0)
    {
        /* Load timer registers */
        TL0 = TL0TMP;
        TH0 = TH0TMP;

        /* Start Timer0 */
        set_TCON_TR0;

        /* Wait for timer overflow */
        while (!TF0);

        /* Clear timer overflow flag */
        clr_TCON_TF0;

        /* Stop Timer0 */
        clr_TCON_TR0;

        /* Decrement counter */
        u16CNT--;
    }
}

/**
 * @brief Simple software delay (less accurate than Timer0_Delay)
 * @param count Number of loop iterations
 *
 * @note This delay is less accurate and depends on compiler optimization level
 * @note Use Timer0_Delay for precise timing
 */
void Simple_Delay(unsigned int count)
{
    while (count--)
    {
        _nop_();
    }
}
