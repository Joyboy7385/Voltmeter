/**
 * ============================================================================
 * VOLTMETER - SDCC VERSION
 * ============================================================================
 * Professional 8051-based dual-channel digital voltmeter with comprehensive
 * error handling, defensive programming, and production-ready robustness.
 *
 * Platform: Nuvoton MS51FB9AE (8051 architecture)
 * Compiler: SDCC (Small Device C Compiler)
 * Version: 2.0 SDCC Port
 * Date: 2025-11-19
 * ============================================================================
 */

#include "ms51_sdcc.h"
#include "sfr_macro_sdcc.h"
#include "function_define_sdcc.h"

/* Forward declaration of delay function */
void Timer0_Delay(unsigned long u32SYSCLK, unsigned int u16CNT, unsigned int u16DLYUnit);

__bit BIT_TMP;  /* Required for SFR macros */

/* ============================================================================
   BUILD CONFIGURATION
   ============================================================================ */

/* Build mode selection - uncomment ONE of these */
/* #define BUILD_PRODUCTION    */ /* Production build with code protection & 220V support */
/* #define BUILD_DEVELOPMENT   */ /* Development build with debugging features */
#define BUILD_STANDARD         /* Standard build for general use */

/* Feature flags based on build mode */
#ifdef BUILD_PRODUCTION
    #define ENABLE_CODE_PROTECTION     1
    #define ENABLE_EEPROM              1
    #define ENABLE_WATCHDOG            1
    #define ENABLE_VOLTAGE_MULTIPLIER  1  /* Hardware divider for 220V */
    #define ENABLE_DEBUG_DISPLAY       0
    #define VOLTAGE_MULTIPLIER_NUM     278UL
    #define VOLTAGE_MULTIPLIER_DENOM   100UL
    #define EMA_SHIFT                  3  /* 12.5% new, 87.5% old (1/8) */
    #define HYSTERESIS_THRESHOLD       3  /* 3V threshold */
    #define ADC_SAMPLES                64
    #define UPDATE_RATE_MS             200
#elif defined(BUILD_DEVELOPMENT)
    #define ENABLE_CODE_PROTECTION     0
    #define ENABLE_EEPROM              0
    #define ENABLE_WATCHDOG            0
    #define ENABLE_VOLTAGE_MULTIPLIER  0
    #define ENABLE_DEBUG_DISPLAY       1
    #define EMA_SHIFT                  1  /* 50% new, 50% old (1/2) */
    #define HYSTERESIS_THRESHOLD       1  /* 1V threshold */
    #define ADC_SAMPLES                16
    #define UPDATE_RATE_MS             150
#else /* BUILD_STANDARD */
    #define ENABLE_CODE_PROTECTION     0
    #define ENABLE_EEPROM              1
    #define ENABLE_WATCHDOG            0  /* Disabled for initial testing */
    #define ENABLE_VOLTAGE_MULTIPLIER  0
    #define ENABLE_DEBUG_DISPLAY       0
    #define EMA_SHIFT                  1  /* 50% new, 50% old (1/2) */
    #define HYSTERESIS_THRESHOLD       2  /* 2V threshold */
    #define ADC_SAMPLES                32
    #define UPDATE_RATE_MS             150
#endif

/* ============================================================================
   SYSTEM CONSTANTS
   ============================================================================ */

/* Hardware configuration */
#define FSYS_CLOCK              16000000UL
#define ADC_RESOLUTION          4096UL
#define ADC_BANDGAP_MV          1220UL
#define ADC_TIMEOUT             5000
#define MAX_DISPLAY_VALUE       999

/* Timing constants */
#define DEBOUNCE_MS             20
#define CAL_TIMEOUT_MS          2000
#define VDD_CACHE_MS            1000      /* Cache VDD for 1 second */
#define WATCHDOG_KICK_MS        100       /* Kick watchdog every 100ms */

/* Calibration storage */
#define MAX_CALIBRATION         99
#define EEPROM_CAL_ADDR_OUT     0x00
#define EEPROM_CAL_ADDR_IN      0x02
#define EEPROM_SIGNATURE        0xA5

/* Voltage calculation constants */
#define VOLTAGE_SCALE_NUM       100UL
#define VOLTAGE_SCALE_DENOM     (ADC_RESOLUTION * 1000UL)
#define ROUNDING_OFFSET         (VOLTAGE_SCALE_DENOM / 2)

/* ADC validation thresholds */
#define ADC_MIN_VALID           5         /* Below this = likely error */
#define ADC_MAX_VALID           4090      /* Above this = likely saturation */
#define ADC_OUTLIER_THRESHOLD   500       /* Max deviation from avg */
#define VDD_MIN_VALID           4000      /* 4.0V minimum */
#define VDD_MAX_VALID           6000      /* 6.0V maximum */

/* Error codes for display */
#define ERROR_NONE              0
#define ERROR_ADC_TIMEOUT       1
#define ERROR_ADC_INVALID       2
#define ERROR_VDD_OUT_OF_RANGE  3
#define ERROR_INIT_FAILED       4

/* ============================================================================
   7-SEGMENT DISPLAY PATTERNS
   ============================================================================ */

static const unsigned char __code DIGIT_PAT[10] = {
    0x3F, /*0*/ 0x06, /*1*/ 0x5B, /*2*/ 0x4F, /*3*/ 0x66, /*4*/
    0x6D, /*5*/ 0x7D, /*6*/ 0x07, /*7*/ 0x7F, /*8*/ 0x6F  /*9*/
};

#define PATTERN_O       0x3F
#define PATTERN_P       0x73
#define PATTERN_I       0x06
#define PATTERN_DASH    0x40
#define PATTERN_E       0x79  /* E for Error */
#define PATTERN_r       0x50  /* r for Error */

#define CA_OUTPUT(pat)  ((~(pat)) & 0x7F)

/* ============================================================================
   PIN DEFINITIONS (using SDCC __sbit)
   ============================================================================ */

__sbit __at (0x90) DIGIT1;  /* P1.0 - Hundreds digit */
__sbit __at (0x91) DIGIT2;  /* P1.1 - Tens digit */
__sbit __at (0x92) DIGIT3;  /* P1.2 - Ones digit */

__sbit __at (0xA0) BTN_INC; /* P2.0 - Increment button (active-low) */
__sbit __at (0x95) BTN_DEC; /* P1.5 - Decrement button (active-low) */

/* ============================================================================
   GLOBAL STATE
   ============================================================================ */

/* Display buffer */
static unsigned char segPattern[3] = {0, 0, 0};

/* Calibration offsets */
static int cal_offset_out = 0;
static int cal_offset_in  = 0;

/* Button debounce state */
static unsigned char inc_stable = 1, inc_cnt = 0;
static unsigned char dec_stable = 1, dec_cnt = 0;

/* EMA filter state */
static unsigned int ema_filtered[2] = {0, 0};
static unsigned char ema_initialized[2] = {0, 0};

/* VDD caching */
static unsigned int vdd_cached = 0;
static unsigned int vdd_cache_timer = 0;

/* Error tracking */
static unsigned char last_error = ERROR_NONE;
static unsigned char error_count = 0;

/* Watchdog timer counter */
static unsigned int watchdog_timer = 0;

/* ============================================================================
   WATCHDOG TIMER FUNCTIONS
   ============================================================================ */

#if ENABLE_WATCHDOG
/**
 * @brief Initialize watchdog timer
 * @note Sets WDT timeout to ~1.6 seconds at 16MHz
 */
static void Watchdog_Init(void)
{
    /* Configure WDT:
     * WDCON[2:0] = prescaler bits
     * 111b = longest timeout (~1.6s at 16MHz)
     */
    TA = 0xAA;  /* Timed access */
    TA = 0x55;
    WDCON = 0x07;  /* Set prescaler to maximum */

    /* Enable WDT */
    set_WDCON_WDTR;  /* Clear WDT counter */
    set_WDCON_WDCLR; /* Start WDT */
}

/**
 * @brief Service (kick) the watchdog timer
 * @note Must be called at least every 1.6 seconds
 */
static void Watchdog_Kick(void)
{
    set_WDCON_WDTR;  /* Clear WDT counter */
}
#endif

/* ============================================================================
   ERROR HANDLING FUNCTIONS
   ============================================================================ */

/**
 * @brief Set error code and increment error counter
 * @param error_code Error code to set
 */
static void Set_Error(unsigned char error_code)
{
    last_error = error_code;
    if (error_count < 255) {
        error_count++;
    }
}

/**
 * @brief Clear error state
 */
static void Clear_Error(void)
{
    last_error = ERROR_NONE;
}

/**
 * @brief Display error code on 7-segment display
 * @param error_code Error code to display (0-9)
 * @note Displays as "Err" on first read, "Exx" on subsequent reads
 */
static void Display_Error(unsigned char error_code)
{
    segPattern[0] = PATTERN_E;
    segPattern[1] = PATTERN_r;
    segPattern[2] = (error_code < 10) ? DIGIT_PAT[error_code] : DIGIT_PAT[9];
}

/* ============================================================================
   EEPROM FUNCTIONS
   ============================================================================ */

#if ENABLE_EEPROM
/**
 * @brief Write byte to EEPROM with verification
 * @param addr EEPROM address
 * @param dat Data byte to write
 * @return 1 if successful, 0 if verification failed
 */
static unsigned char EEPROM_Write_Verify(unsigned int addr, unsigned char dat)
{
    unsigned char verify;

    /* Write data */
    set_CHPCON_IAPEN;
    IAPCN = 0x21;  /* EEPROM program */
    IAPAL = (unsigned char)(addr & 0xFF);
    IAPAH = (unsigned char)(addr >> 8);
    IAPFD = dat;
    set_IAPTRG_IAPGO;
    while(CHPCON & 0x01);  /* Wait for completion */
    clr_CHPCON_IAPEN;

    /* Verify write */
    set_CHPCON_IAPEN;
    IAPCN = 0x00;  /* EEPROM read */
    IAPAL = (unsigned char)(addr & 0xFF);
    IAPAH = (unsigned char)(addr >> 8);
    set_IAPTRG_IAPGO;
    verify = IAPFD;
    clr_CHPCON_IAPEN;

    return (verify == dat) ? 1 : 0;
}

/**
 * @brief Read byte from EEPROM
 * @param addr EEPROM address
 * @return Data byte
 */
static unsigned char EEPROM_Read(unsigned int addr)
{
    unsigned char dat;
    set_CHPCON_IAPEN;
    IAPCN = 0x00;
    IAPAL = (unsigned char)(addr & 0xFF);
    IAPAH = (unsigned char)(addr >> 8);
    set_IAPTRG_IAPGO;
    dat = IAPFD;
    clr_CHPCON_IAPEN;
    return dat;
}

/**
 * @brief Save calibration with validation
 */
static void Save_Calibration(void)
{
    /* Save with bias to handle negative values */
    EEPROM_Write_Verify(EEPROM_CAL_ADDR_OUT, (unsigned char)(cal_offset_out + 99));
    EEPROM_Write_Verify(EEPROM_CAL_ADDR_OUT + 1, EEPROM_SIGNATURE);
    EEPROM_Write_Verify(EEPROM_CAL_ADDR_IN, (unsigned char)(cal_offset_in + 99));
    EEPROM_Write_Verify(EEPROM_CAL_ADDR_IN + 1, EEPROM_SIGNATURE);
}

/**
 * @brief Load calibration with validation
 */
static void Load_Calibration(void)
{
    /* Load O/P calibration */
    if (EEPROM_Read(EEPROM_CAL_ADDR_OUT + 1) == EEPROM_SIGNATURE) {
        cal_offset_out = (int)EEPROM_Read(EEPROM_CAL_ADDR_OUT) - 99;
        /* Validate range */
        if (cal_offset_out > MAX_CALIBRATION || cal_offset_out < -MAX_CALIBRATION) {
            cal_offset_out = 0;
        }
    }

    /* Load I/P calibration */
    if (EEPROM_Read(EEPROM_CAL_ADDR_IN + 1) == EEPROM_SIGNATURE) {
        cal_offset_in = (int)EEPROM_Read(EEPROM_CAL_ADDR_IN) - 99;
        /* Validate range */
        if (cal_offset_in > MAX_CALIBRATION || cal_offset_in < -MAX_CALIBRATION) {
            cal_offset_in = 0;
        }
    }
}
#endif

/* ============================================================================
   BUTTON HANDLING
   ============================================================================ */

static void Buttons_Update(unsigned char *inc_press, unsigned char *dec_press)
{
    unsigned char raw;
    *inc_press = 0;
    *dec_press = 0;

    /* INC button debouncing */
    raw = BTN_INC;
    if (raw != inc_stable) {
        if (++inc_cnt >= DEBOUNCE_MS) {
            inc_stable = raw;
            inc_cnt = 0;
            if (inc_stable == 0) *inc_press = 1;
        }
    } else {
        inc_cnt = 0;
    }

    /* DEC button debouncing */
    raw = BTN_DEC;
    if (raw != dec_stable) {
        if (++dec_cnt >= DEBOUNCE_MS) {
            dec_stable = raw;
            dec_cnt = 0;
            if (dec_stable == 0) *dec_press = 1;
        }
    } else {
        dec_cnt = 0;
    }
}

/* ============================================================================
   ADC FUNCTIONS WITH ERROR HANDLING
   ============================================================================ */

static unsigned char ADC_InitChannels(void)
{
    /* Configure analog input pins */
    P17_Input_Mode;  /* AIN0 */
    P30_Input_Mode;  /* AIN1 */

    /* Disable digital input buffers */
    AINDIDS |= SET_BIT0;  /* P1.7 */
    AINDIDS |= SET_BIT1;  /* P3.0 */

    /* Configure ADC clock: Fsys/8 = 2MHz */
    ADCCON1 = 0x30;
    ADCCON1 |= 0x01;  /* Enable ADC */

    /* Set sampling time */
    ADCDLY = 0x80;

    /* Verify ADC is enabled */
    if (!(ADCCON1 & 0x01)) return 0;
    return 1;
}

static unsigned char ADC_ReadChannel_Safe(unsigned char ch, unsigned int *result)
{
    unsigned int timeout;
    unsigned int adc_val;

    if (ch > 15) {
        Set_Error(ERROR_ADC_INVALID);
        return 0;
    }

    /* Select channel */
    ADCCON0 &= 0xF0;
    ADCCON0 |= (ch & 0x0F);

    /* Start conversion */
    clr_ADCF;
    set_ADCS;

    /* Wait for completion with timeout */
    timeout = ADC_TIMEOUT;
    while (ADCF == 0 && --timeout);

    if (timeout == 0) {
        Set_Error(ERROR_ADC_TIMEOUT);
        *result = 0;
        return 0;
    }

    /* Read 12-bit result */
    adc_val = ADCRH;
    adc_val = (adc_val << 4) | (ADCRL & 0x0F);

    if (adc_val < ADC_MIN_VALID || adc_val > ADC_MAX_VALID) {
        if (ch != 8) {
            Set_Error(ERROR_ADC_INVALID);
            *result = adc_val;
            return 0;
        }
    }

    *result = adc_val;
    return 1;
}

static unsigned char Measure_VDD_mV_Cached(unsigned int *vdd_mV)
{
    unsigned long sum = 0;
    unsigned int adc_bg;
    unsigned char i, valid_samples = 0;
    unsigned int temp_vdd;

    if (vdd_cache_timer > 0 && vdd_cached != 0) {
        *vdd_mV = vdd_cached;
        return 1;
    }

    for (i = 0; i < 16; i++) {
        unsigned int sample;
        if (ADC_ReadChannel_Safe(8, &sample)) {
            if (sample > 100 && sample < 4000) {
                sum += sample;
                valid_samples++;
            }
        }
    }

    if (valid_samples < 8) {
        Set_Error(ERROR_VDD_OUT_OF_RANGE);
        *vdd_mV = 5000;
        return 0;
    }

    adc_bg = (unsigned int)(sum / valid_samples);
    if (adc_bg == 0) adc_bg = 1;

    temp_vdd = (ADC_BANDGAP_MV * ADC_RESOLUTION) / adc_bg;

    if (temp_vdd < VDD_MIN_VALID || temp_vdd > VDD_MAX_VALID) {
        Set_Error(ERROR_VDD_OUT_OF_RANGE);
        *vdd_mV = 5000;
        return 0;
    }

    vdd_cached = temp_vdd;
    vdd_cache_timer = VDD_CACHE_MS;
    *vdd_mV = temp_vdd;
    return 1;
}

static unsigned char ReadAverageVoltage_Safe(unsigned char channel, unsigned char samples, unsigned int *voltage)
{
    unsigned long sum = 0;
    unsigned char i, valid_samples = 0;
    unsigned int avg, vdd_mV;
    unsigned long vt;
    unsigned int volts, first_sample = 0;

    for (i = 0; i < samples; i++) {
        unsigned int sample;
        if (ADC_ReadChannel_Safe(channel, &sample)) {
            if (valid_samples == 0) {
                first_sample = sample;
                sum += sample;
                valid_samples++;
            } else {
                int diff = (int)sample - (int)first_sample;
                if (diff < 0) diff = -diff;
                if ((unsigned int)diff < ADC_OUTLIER_THRESHOLD) {
                    sum += sample;
                    valid_samples++;
                }
            }
        }
    }

    if (valid_samples < (samples >> 2)) {
        Set_Error(ERROR_ADC_INVALID);
        *voltage = 0;
        return 0;
    }

    avg = (unsigned int)(sum / valid_samples);

    if (!Measure_VDD_mV_Cached(&vdd_mV)) {
        *voltage = 0;
        return 0;
    }

    vt = (unsigned long)avg * vdd_mV * VOLTAGE_SCALE_NUM;
    volts = (vt + ROUNDING_OFFSET) / VOLTAGE_SCALE_DENOM;

#if ENABLE_VOLTAGE_MULTIPLIER
    volts = (volts * VOLTAGE_MULTIPLIER_NUM) / VOLTAGE_MULTIPLIER_DENOM;
#endif

    if (volts > MAX_DISPLAY_VALUE) volts = MAX_DISPLAY_VALUE;

    if (!ema_initialized[channel]) {
        ema_filtered[channel] = volts;
        ema_initialized[channel] = 1;
    } else {
        unsigned long temp = volts + (((1UL << EMA_SHIFT) - 1) * ema_filtered[channel]);
        ema_filtered[channel] = (unsigned int)(temp >> EMA_SHIFT);
    }

    {
        static unsigned int last_display[2] = {0, 0};
        unsigned int filtered = ema_filtered[channel];
        int diff;

        if (!last_display[channel]) last_display[channel] = filtered;

        diff = (int)filtered - (int)last_display[channel];
        if (diff < 0) diff = -diff;

        if ((unsigned int)diff >= HYSTERESIS_THRESHOLD) last_display[channel] = filtered;
        *voltage = last_display[channel];
    }

    Clear_Error();
    return 1;
}

/* ============================================================================
   DISPLAY FUNCTIONS
   ============================================================================ */

static void UpdateDisplayBufferForValue(unsigned int value)
{
    unsigned int h, t, o;

    if (value > MAX_DISPLAY_VALUE) value = MAX_DISPLAY_VALUE;

    h = value / 100U;
    t = (value / 10U) % 10U;
    o = value % 10U;

    segPattern[0] = (h == 0) ? 0x00 : DIGIT_PAT[h];
    segPattern[1] = (h == 0 && t == 0) ? 0x00 : DIGIT_PAT[t];
    segPattern[2] = DIGIT_PAT[o];
}

static void UpdateDisplayBufferForLabel(char first, char second)
{
    segPattern[0] = PATTERN_DASH;
    segPattern[1] = (first == 'O') ? PATTERN_O : (first == 'I') ? PATTERN_I : 0x00;
    segPattern[2] = (second == 'P') ? PATTERN_P : 0x00;
}

static void Display_Refresh_1ms(unsigned char *digit_idx)
{
    DIGIT1 = 0;
    DIGIT2 = 0;
    DIGIT3 = 0;

    if (*digit_idx == 0) {
        P0 = CA_OUTPUT(segPattern[0]);
        DIGIT1 = 1;
    } else if (*digit_idx == 1) {
        P0 = CA_OUTPUT(segPattern[1]);
        DIGIT2 = 1;
    } else {
        P0 = CA_OUTPUT(segPattern[2]);
        DIGIT3 = 1;
    }

    Timer0_Delay(FSYS_CLOCK, 1, 1000);

    if (*digit_idx == 0) {
        DIGIT1 = 0;
    } else if (*digit_idx == 1) {
        DIGIT2 = 0;
    } else {
        DIGIT3 = 0;
    }

    (*digit_idx)++;
    if (*digit_idx > 2) *digit_idx = 0;
}

static void DelayWithDisplay(unsigned int ms)
{
    unsigned char d = 0;
    while (ms--) {
        Display_Refresh_1ms(&d);

#if ENABLE_WATCHDOG
        if (++watchdog_timer >= WATCHDOG_KICK_MS) {
            watchdog_timer = 0;
            Watchdog_Kick();
        }
#endif

        if (vdd_cache_timer > 0) vdd_cache_timer--;
    }
}

static unsigned int ApplyOffset(unsigned int v, int offset)
{
    int adj = (int)v + offset;
    if (adj < 0) adj = 0;
    if (adj > MAX_DISPLAY_VALUE) adj = MAX_DISPLAY_VALUE;
    return (unsigned int)adj;
}

static void ShowVoltageWithCalibration(unsigned char channel, unsigned int normal_ms, int *offset_ptr)
{
    unsigned char digit = 0;
    unsigned int update_tick = 0;
    unsigned char inc_evt, dec_evt;
    __bit cal_mode = 0;
    unsigned int idle_ms = 0;
    unsigned int v = 0;

    if (!ReadAverageVoltage_Safe(channel, ADC_SAMPLES, &v)) {
#if ENABLE_DEBUG_DISPLAY
        Display_Error(last_error);
        DelayWithDisplay(2000);
        return;
#else
        v = 0;
#endif
    }

    v = ApplyOffset(v, *offset_ptr);
    UpdateDisplayBufferForValue(v);

    while (1) {
        Display_Refresh_1ms(&digit);

        if (++update_tick >= UPDATE_RATE_MS) {
            update_tick = 0;
            if (ReadAverageVoltage_Safe(channel, ADC_SAMPLES, &v)) {
                v = ApplyOffset(v, *offset_ptr);
                UpdateDisplayBufferForValue(v);
            }
#if ENABLE_DEBUG_DISPLAY
            else {
                Display_Error(last_error);
            }
#endif
        }

        Buttons_Update(&inc_evt, &dec_evt);

        if (inc_evt) {
            (*offset_ptr)++;
            if (*offset_ptr > MAX_CALIBRATION) *offset_ptr = MAX_CALIBRATION;
            cal_mode = 1;
            idle_ms = CAL_TIMEOUT_MS;
            update_tick = UPDATE_RATE_MS;
        }

        if (dec_evt) {
            (*offset_ptr)--;
            if (*offset_ptr < -MAX_CALIBRATION) *offset_ptr = -MAX_CALIBRATION;
            cal_mode = 1;
            idle_ms = CAL_TIMEOUT_MS;
            update_tick = UPDATE_RATE_MS;
        }

#if ENABLE_WATCHDOG
        if (++watchdog_timer >= WATCHDOG_KICK_MS) {
            watchdog_timer = 0;
            Watchdog_Kick();
        }
#endif

        if (vdd_cache_timer > 0) vdd_cache_timer--;

        if (cal_mode) {
            if (idle_ms) {
                idle_ms--;
            } else {
#if ENABLE_EEPROM
                Save_Calibration();
#endif
                break;
            }
        } else {
            if (normal_ms) {
                normal_ms--;
            } else {
                break;
            }
        }
    }
}

/* ============================================================================
   INITIALIZATION
   ============================================================================ */

static unsigned char IO_Init(void)
{
    /* Disable RESET on P2.0 (for INC button) */
    TA = 0xAA;
    TA = 0x55;
    AUXR0 &= ~0x04;

    /* Configure segment outputs (P0.0-P0.6) */
    P00_PUSHPULL_MODE; P01_PUSHPULL_MODE; P02_PUSHPULL_MODE;
    P03_PUSHPULL_MODE; P04_PUSHPULL_MODE; P05_PUSHPULL_MODE;
    P06_PUSHPULL_MODE;

    /* Configure digit selectors (P1.0-P1.2) */
    P10_PUSHPULL_MODE; P11_PUSHPULL_MODE; P12_PUSHPULL_MODE;

    /* Configure DEC button (P1.5) */
    P15_QUASI_MODE;

    /* Initialize outputs */
    DIGIT1 = 0;
    DIGIT2 = 0;
    DIGIT3 = 0;
    P0 = 0xFF;

    return 1;
}

static unsigned char System_Init(void)
{
    unsigned char init_ok = 1;

    if (!IO_Init()) init_ok = 0;
    if (!ADC_InitChannels()) {
        Set_Error(ERROR_INIT_FAILED);
        init_ok = 0;
    }

#if ENABLE_EEPROM
    Load_Calibration();
#endif

#if ENABLE_WATCHDOG
    Watchdog_Init();
#endif

    return init_ok;
}

/* ============================================================================
   MAIN
   ============================================================================ */

void main(void)
{
    unsigned char init_ok;

    init_ok = System_Init();

    if (!init_ok) {
        Display_Error(ERROR_INIT_FAILED);
        DelayWithDisplay(3000);
    }

    while (1) {
#if ENABLE_WATCHDOG
        Watchdog_Kick();
#endif

        UpdateDisplayBufferForLabel('O', 'P');
        DelayWithDisplay(1000);

        ShowVoltageWithCalibration(0, 3000, &cal_offset_out);

        UpdateDisplayBufferForLabel('I', 'P');
        DelayWithDisplay(1000);

        ShowVoltageWithCalibration(1, 3000, &cal_offset_in);
    }
}
