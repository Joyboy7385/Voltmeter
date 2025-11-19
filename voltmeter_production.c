
/*#include "ms51_16k_keil.h"               // Base SFR register definitions */
#include "SFR_Macro_MS51_16K_keil.h"     // Low-level SFR macros (set/clear bits)
//#include "Function_define_MS51_16K_keil.h" // I/O and functional macros
//#include "Delay_keil.h"                  // Delay function declarations

bit BIT_TMP;   // definition for the macros in Function_define_MS51_16K_keil.h

/* =========================
   Build Configuration
   ========================= */
// Uncomment the following line for production builds with code protection
// #define PRODUCTION_BUILD

#ifdef PRODUCTION_BUILD
    #warning "PRODUCTION BUILD: Code protection will be enabled!"
    #warning "Make sure all testing is complete before flashing!"
    #define ENABLE_CODE_PROTECTION  1
    #define ENABLE_DEBUG            0
#else
    #define ENABLE_CODE_PROTECTION  0
    #define ENABLE_DEBUG            1
#endif

/* =========================
   System Configuration
   ========================= */
#define FSYS_CLOCK            16000000UL   // 16 MHz system clock
#define ADC_RESOLUTION        4096UL       // 12-bit ADC (0-4095)
#define ADC_BANDGAP_MV        1220UL       // Internal bandgap voltage in mV
#define ADC_TIMEOUT           5000         // ADC conversion timeout counter

/* Display constants */
#define MAX_DISPLAY_VALUE     999          // Maximum displayable value
#define DEBOUNCE_MS           20           // Button debounce time in ms

/* Calibration constants */
#define MAX_CALIBRATION       99           // Maximum calibration offset (+/- volts)
#define CAL_TIMEOUT_MS        2000         // Calibration mode timeout
#define EEPROM_CAL_ADDR_OUT   0x00         // EEPROM address for O/P calibration
#define EEPROM_CAL_ADDR_IN    0x02         // EEPROM address for I/P calibration

/* Voltage calculation constants */
#define VOLTAGE_SCALE_NUM     100UL        // Scaling numerator (see algorithm below)
#define VOLTAGE_SCALE_DENOM   (ADC_RESOLUTION * 1000UL)  // Scaling denominator = 4,096,000
#define ROUNDING_OFFSET       (VOLTAGE_SCALE_DENOM / 2)  // For proper rounding = 2,048,000

/* Hardware voltage divider compensation for 220V mains measurement
 * Hardware divider scales 220V down to ~3.7V for ADC input
 * Measured ratio: 220V / 85 display units ≈ 2.59
 * Fixed-point representation: multiply by 259, divide by 100
 */
#define VOLTAGE_MULTIPLIER_NUM    278UL    // Multiplier numerator (2.78 * 100) - Calibrated for actual hardware divider ratio
#define VOLTAGE_MULTIPLIER_DENOM  100UL    // Multiplier denominator

/* Security constants */
#define FIRMWARE_VERSION      0x0100       // Version 1.0
#define PRODUCT_ID            0x5A3C       // Unique product identifier

/**
 * Voltage Calculation Algorithm (for 220V AC Mains Measurement):
 * =================================================================
 * Hardware voltage divider scales 220V down to ~3.7V for ADC input.
 * After ADC conversion and VDD compensation, software applies multiplier (x2.59)
 * to compensate for hardware divider ratio.
 *
 * Display shows voltage in whole volts (1V units):
 *   Display=100 means 100V
 *   Display=220 means 220V
 *   Display=999 means 999V (maximum)
 *
 * Step 1: ADC conversion and VDD compensation
 * --------------------------------------------
 * ADC_VALUE / 4096 = V_input / V_DD
 * V_input = (ADC_VALUE * V_DD) / 4096
 *
 * To display in centivolt units (multiply voltage by 100):
 * display = V_input * 100
 * display = ((ADC * VDD_mV) / 1000 / 4096) * 100
 * display = (ADC * VDD_mV * 100) / (4096 * 1000)
 * display = (ADC * VDD_mV) / (4096 * 10)
 * display = (ADC * VDD_mV) / 40960
 *
 * To maintain precision with integer math:
 * vt = ADC * VDD_mV * 100
 * display = (vt + rounding) / 4,096,000
 *
 * Step 2: Hardware voltage divider compensation
 * ----------------------------------------------
 * Actual_voltage = measured_voltage * 2.59
 * display_actual = (display * 259) / 100
 *
 * Step 3: EMA filtering
 * ---------------------
 * filtered = (1 * new + 3 * old) / 4  [25% new, 75% old for stability]
 *
 * Rounding offset is half the divisor: 4,096,000 / 2 = 2,048,000
 * But to avoid overflow, we use: 4096 / 2 = 2048 (works due to order of operations)
 */

/* =========================
   7-seg (common-cathode codes)
   ========================= */
static const unsigned char DIGIT_PAT[10] = {
    0x3F, /*0*/ 0x06, /*1*/ 0x5B, /*2*/ 0x4F, /*3*/ 0x66, /*4*/
    0x6D, /*5*/ 0x7D, /*6*/ 0x07, /*7*/ 0x7F, /*8*/ 0x6F  /*9*/
};
#define PATTERN_O    0x3F
#define PATTERN_P    0x63
#define PATTERN_I    0x06
#define PATTERN_DASH 0x40

/* Common-anode: drive segments LOW to light, so invert the 7 bits */
#define CA_OUTPUT(pat) ((~(pat)) & 0x7F)

/* =========================
   Pin Mapping (MS51FB9AE)
   =========================
   Segments (cathodes, active-LOW) on P0:
     a->P0.0  b->P0.1  c->P0.2  d->P0.3  e->P0.4  f->P0.5  g->P0.6
     (DP unused, P0.7 free)

   Digit anodes (active-HIGH) on P1:
     DIGIT1 (hundreds)->P1.0, DIGIT2 (tens)->P1.1, DIGIT3 (ones)->P1.2

   ADC channels:
     O/P (Output)  -> AIN0 = P1.7
     I/P (Input)   -> AIN1 = P3.0

   Calibration buttons (active-LOW to GND with internal pull-ups):
     INC (+) -> P2.0
     DEC (-) -> P1.5
*/
sbit DIGIT1 = P1^0;
sbit DIGIT2 = P1^1;
sbit DIGIT3 = P1^2;

sbit BTN_INC = P2^0;   /* pull-up (quasi), press = 0 */
sbit BTN_DEC = P1^5;   /* pull-up (quasi), press = 0 */

/* Display buffer (common-cathode codes) */
static unsigned char segPattern[3] = {0,0,0};

/* Per-channel calibration offsets in volts (signed) */
static int cal_offset_out = 0;   /* applies when O/P showing (AIN0) */
static int cal_offset_in  = 0;   /* applies when I/P showing (AIN1) */

/* Button debounce state */
static unsigned char inc_stable = 1, inc_cnt = 0;
static unsigned char dec_stable = 1, dec_cnt = 0;

/* =========================
   Code Protection Functions
   ========================= */

#if ENABLE_CODE_PROTECTION
/**
 * @brief Enable code protection to prevent code extraction
 *
 * This function locks the flash memory by setting the LOCK bit in CONFIG0.
 *
 * WARNING: After calling this function and power cycling:
 * - Flash memory cannot be read via ISP/ICP
 * - On-chip debugging is disabled
 * - Code can only be erased via mass erase (destroys all code)
 *
 * @note Only call this in production builds!
 * @note Chip must be power-cycled for lock to take effect
 */
static void Enable_Code_Protection(void)
{
    /* Enable IAP (In-Application Programming) */
    set_CHPCON_IAPEN;

    /* Enable CONFIG byte modification */
    IAPUEN = 0x01;

    /* Prepare to write CONFIG0 register */
    IAPCN = 0xE1;   /* CONFIG write command */
    IAPAL = 0x00;   /* CONFIG0 address low byte */
    IAPAH = 0x00;   /* CONFIG0 address high byte */

    /* Write CONFIG0 with LOCK=0 (locked) and OCDEN=0 (debug disabled) */
    IAPFD = 0x7E;   /* CONFIG0 = 0x7E (LOCK and OCDEN disabled) */

    /* Trigger IAP write operation */
    set_IAPTRG_IAPGO;

    /* Wait for write to complete */
    while (CHPCON & 0x01);  /* Wait for IAPGO to clear */

    /* Disable IAP */
    clr_CHPCON_IAPEN;

    /* Code protection will be active after next power cycle */
}

/**
 * @brief Check if code protection is already enabled
 * @return 1 if locked, 0 if unlocked
 */
static unsigned char Is_Code_Protected(void)
{
    /* Enable IAP for CONFIG read */
    set_CHPCON_IAPEN;
    IAPUEN = 0x01;

    /* Read CONFIG0 */
    IAPCN = 0xC0;   /* CONFIG read command */
    IAPAL = 0x00;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;

    /* Get CONFIG0 value */
    unsigned char config0 = IAPFD;

    /* Disable IAP */
    clr_CHPCON_IAPEN;

    /* Check LOCK bit (bit 7): 0=locked, 1=unlocked */
    return (config0 & 0x80) ? 0 : 1;
}
#endif

/**
 * @brief Update button debounce state and detect new presses
 * @param[out] inc_press Set to 1 if INC button newly pressed
 * @param[out] dec_press Set to 1 if DEC button newly pressed
 *
 * @note Call this function once per millisecond for proper debouncing
 * @note Buttons are active-LOW (pressed = 0)
 */
static void Buttons_Update(unsigned char *inc_press, unsigned char *dec_press)
{
    unsigned char raw;
    *inc_press = 0; *dec_press = 0;

    /* Process INC button */
    raw = BTN_INC; /* 1=idle, 0=pressed */
    if (raw != inc_stable) {
        if (++inc_cnt >= DEBOUNCE_MS) {
            inc_stable = raw;
            inc_cnt = 0;
            if (inc_stable == 0)
                *inc_press = 1; /* new stable press */
        }
    } else {
        inc_cnt = 0;
    }

    /* Process DEC button */
    raw = BTN_DEC;
    if (raw != dec_stable) {
        if (++dec_cnt >= DEBOUNCE_MS) {
            dec_stable = raw;
            dec_cnt = 0;
            if (dec_stable == 0)
                *dec_press = 1;
        }
    } else {
        dec_cnt = 0;
    }
}

/* ===== ADC setup & read ===== */

/**
 * @brief Initialize ADC channels and configuration
 *
 * Configures:
 * - P1.7 as AIN0 (O/P channel)
 * - P3.0 as AIN1 (I/P channel)
 * - ADC clock divider for optimal conversion time
 * - ADC sampling time
 *
 * @note For 16MHz Fsys, ADC clock = Fsys/8 = 2MHz (within 0.5-2MHz spec)
 * @note Typical conversion time: ~20us at 2MHz ADC clock
 */
static void ADC_InitChannels(void)
{
    /* Configure analog input pins */
    P17_Input_Mode; /* AIN0 - O/P */
    P30_Input_Mode; /* AIN1 - I/P */

    /* Disable digital input buffers on ADC pins (reduces leakage/noise) */
    AINDIDS |= SET_BIT0; /* P1.7 digital input disable */
    AINDIDS |= SET_BIT1; /* P3.0 digital input disable */

    /* Configure ADC:
     * ADCCON1[7:4] = ADC clock divider
     * For Fsys = 16MHz, divider = 8 gives ADC clock = 2MHz (optimal)
     * ADCCON1[3:0] = Control bits, bit 0 = ADC enable
     */
    ADCCON1 = 0x30;  /* ADC clock = Fsys/8, ADC disabled initially */
    ADCCON1 |= 0x01; /* Enable ADC module */

    /* Configure ADC sampling time (ADCDLY register)
     * Higher value = longer sampling time = better accuracy
     * 0x80 = 128 ADC clock cycles sampling time (~64us at 2MHz ADC clock)
     * Balanced for accuracy and speed
     */
    ADCDLY = 0x80;
}

/**
 * @brief Read ADC channel with timeout protection
 * @param ch ADC channel number (0-15)
 * @return 12-bit ADC result (0-4095), or 0 on timeout
 *
 * @note Blocking function - waits for conversion complete
 * @note Includes timeout protection to prevent infinite loops
 */
static unsigned int ADC_ReadChannel(unsigned char ch)
{
    unsigned int result;
    unsigned int timeout;

    /* Select ADC channel */
    ADCCON0 &= 0xF0;
    ADCCON0 |= (ch & 0x0F);

    /* Start conversion */
    clr_ADCF;
    set_ADCS;

    /* Wait for conversion with timeout protection */
    timeout = ADC_TIMEOUT;
    while (ADCF == 0 && --timeout);

    if (timeout == 0) {
        /* ADC timeout - return 0 */
        return 0;
    }

    /* Read 12-bit result */
    result  = ADCRH;
    result  = (result << 4) | (ADCRL & 0x0F);
    return result; /* 12-bit: 0..4095 */
}

/* Exponential Moving Average filter state for each channel */
static unsigned int ema_filtered[2] = {0, 0};  /* [0]=AIN0, [1]=AIN1 */
static unsigned char ema_initialized[2] = {0, 0};

/**
 * @brief Measure VDD using internal 1.22V bandgap reference
 * @return VDD in millivolts (e.g., 5000 for 5.00V)
 *
 * @note Uses 16x oversampling for good noise reduction with faster response
 * @note Bandgap reference is on ADC channel 8
 * @note Calculation: VDD = (1.22V * 4096) / ADC_reading
 */
static unsigned int Measure_VDD_mV(void)
{
    unsigned long sum = 0;
    unsigned int adc_bg;
    unsigned char i;
    unsigned char samples = 16;  // Optimized for speed and accuracy
    unsigned int vdd_mV;

    /* Oversample bandgap channel for noise reduction */
    for (i=0; i<samples; i++) {
        sum += ADC_ReadChannel(8);   // internal bandgap = 1.22V
    }
    adc_bg = (unsigned int)(sum / samples);

    /* Avoid divide-by-zero */
    if (adc_bg == 0)
        adc_bg = 1;

    /* Calculate VDD: VDD = 1.22V * 4096 / ADC_BG */
    vdd_mV = (ADC_BANDGAP_MV * ADC_RESOLUTION) / adc_bg;

    return vdd_mV; // e.g., 4980 means 4.98V
}

/**
 * @brief Read voltage from ADC channel with averaging, multiplier, and EMA filtering
 * @param channel ADC channel number (0 or 1)
 * @param samples Number of samples to average (48 recommended for balanced 220V measurement)
 * @return Voltage in volts (0-999)
 *
 * @note Automatically compensates for VDD variations
 * @note Applies hardware voltage divider compensation (x2.59 for 220V measurement)
 * @note Uses 12.5% new + 87.5% old EMA filter for balanced stability
 * @note Uses moderate 3V hysteresis for good stability with responsiveness
 * @note See voltage calculation algorithm at top of file
 */
static unsigned int ReadAverageVoltage(unsigned char channel, unsigned char samples)
{
    unsigned long sum = 0;
    unsigned char i;
    unsigned int avg;
    unsigned int vdd_mV;
    unsigned long vt;
    unsigned int volts;

    /* Oversample ADC channel for noise reduction */
    for (i=0; i<samples; i++) {
        sum += ADC_ReadChannel(channel);
    }
    avg = (unsigned int)(sum / samples);

    /* Measure supply voltage dynamically for VDD compensation */
    vdd_mV = Measure_VDD_mV();  // e.g., 5000 mV

    /* Voltage calculation with scaling and rounding
     * Maximum: 4095 * 6000 * 100 = 2,457,000,000 (fits in unsigned long)
     * DO NOT increase scaling factor without checking overflow!
     */
    vt = (unsigned long)avg * vdd_mV * VOLTAGE_SCALE_NUM;
    volts = (vt + ROUNDING_OFFSET) / VOLTAGE_SCALE_DENOM;

    /* Apply hardware voltage divider compensation
     * Hardware divider scales 220V to ~3.7V, ratio = 2.59
     * volts_actual = volts_measured * 2.59
     */
    volts = (volts * VOLTAGE_MULTIPLIER_NUM) / VOLTAGE_MULTIPLIER_DENOM;

    /* Safety clamp for display range */
    if (volts > MAX_DISPLAY_VALUE)
        volts = MAX_DISPLAY_VALUE;

    /* Apply Exponential Moving Average (EMA) filter for smooth, stable readings
     * EMA formula: filtered = (alpha * new_value) + ((1-alpha) * old_value)
     * Using alpha = 0.125 (1/8) for balanced response with good stability
     * This provides strong noise rejection while maintaining reasonable responsiveness
     * Formula: filtered = (1 * new + 7 * old) / 8
     */
    if (!ema_initialized[channel]) {
        /* First reading - initialize filter with current value */
        ema_filtered[channel] = volts;
        ema_initialized[channel] = 1;
    } else {
        /* EMA filter: 12.5% new value + 87.5% old value
         * Using fixed-point math: (new + 7*old) / 8
         * Balanced filter for medium stability
         */
        ema_filtered[channel] = (volts + (ema_filtered[channel] * 7)) >> 3;
    }

    /* Apply moderate hysteresis for balanced stability
     * 3V threshold provides good noise rejection while maintaining responsiveness
     * This prevents minor fluctuations while still tracking real voltage changes
     */
    {
        static unsigned int last_display[2] = {0, 0};
        unsigned int filtered = ema_filtered[channel];
        int diff;

        if (!last_display[channel]) {
            /* First time - initialize */
            last_display[channel] = filtered;
        }

        diff = (int)filtered - (int)last_display[channel];

        /* Update display if change is >= 3V (medium stability threshold) */
        if (diff >= 3 || diff <= -3) {
            last_display[channel] = filtered;
        }

        return last_display[channel];
    }
}


/* ===== Display helpers ===== */

/**
 * @brief Update display buffer with numeric value
 * @param value Value to display (0-999)
 *
 * @note Leading zeros are suppressed
 * @note Example: 123->"123", 23->" 23", 3->"  3"
 */
static void UpdateDisplayBufferForValue(unsigned int value)
{
    unsigned int h = value / 100U;
    unsigned int t = (value / 10U) % 10U;
    unsigned int o = value % 10U;

    segPattern[0] = (h==0) ? 0x00 : DIGIT_PAT[h];
    segPattern[1] = (h==0 && t==0) ? 0x00 : DIGIT_PAT[t];
    segPattern[2] = DIGIT_PAT[o];
}

/**
 * @brief Update display buffer with label (e.g., "-OP", "-IP")
 * @param first First character ('O' or 'I')
 * @param second Second character ('P')
 */
static void UpdateDisplayBufferForLabel(char first, char second)
{
    segPattern[0] = PATTERN_DASH;
    segPattern[1] = (first=='O') ? PATTERN_O : ((first=='I') ? PATTERN_I : 0x00);
    segPattern[2] = (second=='P')? PATTERN_P : 0x00;
}

/**
 * @brief Multiplex display refresh for 1ms
 * @param digit_idx Pointer to current digit index (0-2), auto-increments
 *
 * @note This function takes approximately 1ms to execute
 * @note Uses Timer0 for precise 1ms delay
 */
static void Display_Refresh_1ms(unsigned char *digit_idx)
{
    /* Turn all digits off to avoid ghosting */
    DIGIT1=0; DIGIT2=0; DIGIT3=0;

    /* Turn on current digit with corresponding segments */
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

    /* 1ms delay for this digit */
    Timer0_Delay(FSYS_CLOCK, 1, 1000);

    /* Turn off current digit */
    if (*digit_idx == 0)
        DIGIT1 = 0;
    else if (*digit_idx == 1)
        DIGIT2 = 0;
    else
        DIGIT3 = 0;

    /* Move to next digit */
    (*digit_idx)++;
    if (*digit_idx > 2)
        *digit_idx = 0;
}

/**
 * @brief Delay while refreshing display
 * @param ms Delay time in milliseconds
 *
 * @note Display remains active during delay
 * @note Buttons are NOT scanned during this delay
 */
static void DelayWithDisplay(unsigned int ms)
{
    unsigned char d = 0;
    while (ms--) {
        Display_Refresh_1ms(&d);
    }
}

/**
 * @brief Apply calibration offset and clamp to valid range
 * @param v Raw voltage value
 * @param offset Calibration offset (-99 to +99)
 * @return Calibrated voltage clamped to 0-999
 */
static unsigned int ApplyOffset(unsigned int v, int offset)
{
    int adj = (int)v + offset;
    if (adj < 0) adj = 0;
    if (adj > MAX_DISPLAY_VALUE) adj = MAX_DISPLAY_VALUE;
    return (unsigned int)adj;
}

/**
 * @brief Show live voltage with calibration capability
 * @param channel ADC channel number (0 or 1)
 * @param normal_ms Display time in normal mode (milliseconds)
 * @param offset_ptr Pointer to calibration offset variable
 *
 * Features:
 * - Displays live voltage reading, updated every 400ms for balanced AC mains measurement
 * - User can press INC/DEC to enter calibration mode
 * - In cal mode: adjusts offset and extends display time
 * - Exits cal mode after 2s of no button presses
 *
 * @note Offset range is limited to +/-99 volts
 * @note Uses 48 samples per reading for balanced noise rejection
 */
static void ShowVoltageWithCalibration(unsigned char channel,
                                       unsigned int normal_ms,
                                       int *offset_ptr)
{
    unsigned char digit = 0;
    unsigned int  update_tick = 0;
    unsigned char inc_evt, dec_evt;
    bit cal_mode = 0;
    unsigned int idle_ms = 0;

    /* Initial reading and display with 48 samples for balanced stability */
    unsigned int v = ReadAverageVoltage(channel, 48);
    v = ApplyOffset(v, *offset_ptr);
    UpdateDisplayBufferForValue(v);

    while (1) {
        /* 1ms display refresh */
        Display_Refresh_1ms(&digit);

        /* Update voltage reading every 400ms for balanced display (AC mains measurement) */
        if (++update_tick >= 400) {
            update_tick = 0;
            v = ReadAverageVoltage(channel, 48);
            v = ApplyOffset(v, *offset_ptr);
            UpdateDisplayBufferForValue(v);
        }

        /* Scan buttons once per ms */
        Buttons_Update(&inc_evt, &dec_evt);

        /* Handle INC button press */
        if (inc_evt) {
            (*offset_ptr)++;
            if (*offset_ptr > MAX_CALIBRATION)
                *offset_ptr = MAX_CALIBRATION;
            cal_mode = 1;
            idle_ms = CAL_TIMEOUT_MS;
            update_tick = 400; /* force immediate display update */
        }

        /* Handle DEC button press */
        if (dec_evt) {
            (*offset_ptr)--;
            if (*offset_ptr < -MAX_CALIBRATION)
                *offset_ptr = -MAX_CALIBRATION;
            cal_mode = 1;
            idle_ms = CAL_TIMEOUT_MS;
            update_tick = 400;
        }

        /* Exit conditions */
        if (cal_mode) {
            if (idle_ms)
                idle_ms--;
            else
                break;  /* 2s timeout - exit calibration mode */
        } else {
            if (normal_ms)
                normal_ms--;
            else
                break;  /* finished normal display duration */
        }
    }
}

/* ===== Init I/O ===== */

/**
 * @brief Initialize I/O pins for display, buttons, and ADC
 *
 * Configures:
 * - P0.0-P0.6: Push-pull outputs for 7-segment segments
 * - P1.0-P1.2: Push-pull outputs for digit selection
 * - P1.5: Quasi-bidirectional for DEC button (with pull-up)
 * - P2.0: Quasi-bidirectional for INC button (default, no config needed)
 * - ADC pins configured separately in ADC_InitChannels()
 */
static void IO_Init(void)
{
    /* Segments P0.0..P0.6 as push-pull outputs */
    P00_PUSHPULL_MODE; P01_PUSHPULL_MODE; P02_PUSHPULL_MODE;
    P03_PUSHPULL_MODE; P04_PUSHPULL_MODE; P05_PUSHPULL_MODE;
    P06_PUSHPULL_MODE;

    /* Digits P1.0..P1.2 as push-pull outputs */
    P10_PUSHPULL_MODE; P11_PUSHPULL_MODE; P12_PUSHPULL_MODE;

    /* Button on P1.5 as QUASI (internal pull-up), press to GND */
    P15_QUASI_MODE;

    /* Note: P2.0 (INC button) is already QUASI by default
     * MS51 has no P2M1/P2M2 registers - P2 is fixed QUASI mode */

    /* ADC pins are set inside ADC_InitChannels() */

    /* Default outputs off */
    DIGIT1=0; DIGIT2=0; DIGIT3=0;
    P0 = 0xFF; /* all segments off (common-anode) */
}

/* =========================
   main()
   ========================= */
void main(void)
{
    /* Initialize hardware */
    IO_Init();
    ADC_InitChannels();

#if ENABLE_CODE_PROTECTION
    /* Check if code protection is already enabled */
    if (!Is_Code_Protected()) {
        /* Flash "888" to indicate code protection will be enabled */
        segPattern[0] = DIGIT_PAT[8];
        segPattern[1] = DIGIT_PAT[8];
        segPattern[2] = DIGIT_PAT[8];
        DelayWithDisplay(2000);

        /* Enable code protection */
        Enable_Code_Protection();

        /* Flash "---" to confirm */
        segPattern[0] = PATTERN_DASH;
        segPattern[1] = PATTERN_DASH;
        segPattern[2] = PATTERN_DASH;
        DelayWithDisplay(2000);

        /* Code protection will be active after next power cycle */
    }
#endif

    /* Service watchdog timer (or disable if not used in production) */
    /* For production: Enable WDT and service periodically */
    /* For testing: Can be left as-is or explicitly disabled */

    /* Main loop - alternates between O/P and I/P channels */
    while (1) {
        /* 1) Show label "-OP" for 1 second */
        UpdateDisplayBufferForLabel('O','P');
        DelayWithDisplay(1000);

        /* 2) Show O/P voltage for 3s (or longer if user calibrates) */
        ShowVoltageWithCalibration(0 /* AIN0 */, 3000, &cal_offset_out);

        /* 3) Show label "-IP" for 1 second */
        UpdateDisplayBufferForLabel('I','P');
        DelayWithDisplay(1000);

        /* 4) Show I/P voltage for 3s (or longer if user calibrates) */
        ShowVoltageWithCalibration(1 /* AIN1 */, 3000, &cal_offset_in);
    }
}
