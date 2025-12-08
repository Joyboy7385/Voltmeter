
/*===================================================================================
 * Professional Voltmeter with Kalman Filter
 * For CH32V003A4M6 (RISC-V)
 * Version 2.0 - Production Ready
 *
 * Hardware:
 *   - Voltage Divider: 1240K / 10K with 10uF filter
 *   - Display: 3-digit 7-segment (Common Anode)
 *   - Buttons: INC (PA1), DEC (PA2) with internal pull-up
 *
 *===================================================================================
 * IMPORTANT: WCH-LinkUtility Option Bytes Configuration (MUST SET BEFORE PROGRAMMING)
 *===================================================================================
 * Open WCH-LinkUtility -> Target -> Option Bytes -> Configure as follows:
 *
 *   +-------------------+------------------+----------------------------------------+
 *   | Option            | Required Setting | Reason                                 |
 *   +-------------------+------------------+----------------------------------------+
 *   | IWDG_SW           | Software         | Watchdog controlled by firmware        |
 *   | RST_MODE          | DISABLED         | PD7 used as DIGIT3 GPIO (CRITICAL!)    |
 *   | STOP_RST          | No Reset         | Optional - prevents unwanted resets    |
 *   | STANDBY_RST       | No Reset         | Optional - prevents unwanted resets    |
 *   | BOOT_MODE         | User Code        | Boot from main flash                   |
 *   +-------------------+------------------+----------------------------------------+
 *
 * WARNING: If RST_MODE is not set to DISABLED, DIGIT3 (PD7) will NOT work!
 *          The display will only show 2 digits instead of 3.
 *
 *===================================================================================
 * Pin Assignment Summary
 *===================================================================================
 *   PA1 - BTN_INC (Increment button, active LOW with pull-up)
 *   PA2 - BTN_DEC (Decrement button, active LOW with pull-up)
 *   PC0 - SEG_G
 *   PC1 - SEG_D
 *   PC2 - SEG_C
 *   PC3 - SEG_E
 *   PC4 - ADC_INPUT (Voltage divider input)
 *   PC6 - SEG_F
 *   PC7 - DIGIT1 (Common anode, active HIGH)
 *   PD1 - SEG_A
 *   PD4 - ADC_OUTPUT (Voltage divider output)
 *   PD5 - DIGIT2 (Common anode, active HIGH)
 *   PD6 - SEG_B
 *   PD7 - DIGIT3 (Common anode, active HIGH) ** Requires RST_MODE=DISABLED **
 *
 *===================================================================================*/
#include "debug.h"
#include "ch32v00x_conf.h"
#include <stdint.h>
#include "ch32v00x_flash.h"

/*===================================================================================
 * Hardware Pin Configuration
 *===================================================================================*/
// Buttons (directly connected to GND when pressed, internal pull-up enabled)
#define BTN_INC_PORT    GPIOA
#define BTN_INC_PIN     GPIO_Pin_1      // PA1 - Increment calibration
#define BTN_DEC_PORT    GPIOA
#define BTN_DEC_PIN     GPIO_Pin_2      // PA2 - Decrement calibration

// ADC Channels (via voltage divider: 1240K + 10K, ratio = 125:1)
#define ADC_CH_OUTPUT   ADC_Channel_7   // PD4 - Output voltage measurement
#define ADC_CH_INPUT    ADC_Channel_2   // PC4 - Input voltage measurement
#define ADC_CH_VREF     ADC_Channel_8   // Internal 1.2V reference for VDD calibration

// 7-Segment pins (accent LOW to turn ON segment for common anode)
#define SEG_A_PORT      GPIOD
#define SEG_A_PIN       GPIO_Pin_1      // PD1 - Segment A (top)
#define SEG_B_PORT      GPIOD
#define SEG_B_PIN       GPIO_Pin_6      // PD6 - Segment B (top-right)
#define SEG_C_PORT      GPIOC
#define SEG_C_PIN       GPIO_Pin_2      // PC2 - Segment C (bottom-right)
#define SEG_D_PORT      GPIOC
#define SEG_D_PIN       GPIO_Pin_1      // PC1 - Segment D (bottom)
#define SEG_E_PORT      GPIOC
#define SEG_E_PIN       GPIO_Pin_3      // PC3 - Segment E (bottom-left)
#define SEG_F_PORT      GPIOC
#define SEG_F_PIN       GPIO_Pin_6      // PC6 - Segment F (top-left)
#define SEG_G_PORT      GPIOC
#define SEG_G_PIN       GPIO_Pin_0      // PC0 - Segment G (middle)

// Digit select (active HIGH to enable digit for common anode display)
#define DIGIT1_PORT     GPIOC
#define DIGIT1_PIN      GPIO_Pin_7      // PC7 - Digit 1 (hundreds)
#define DIGIT2_PORT     GPIOD
#define DIGIT2_PIN      GPIO_Pin_5      // PD5 - Digit 2 (tens)
#define DIGIT3_PORT     GPIOD
#define DIGIT3_PIN      GPIO_Pin_7      // PD7 - Digit 3 (ones) ** RST pin - must disable in Option Bytes! **

/*===================================================================================
 * Configuration Constants (No Magic Numbers)
 *===================================================================================*/
// Display limits
#define MAX_DISPLAY_VALUE       999
#define OVERLOAD_THRESHOLD      990     // Show "OL" above this

// Timing constants
#define DEBOUNCE_COUNTS         10      // ~20ms at 2ms/call
#define CAL_TIMEOUT_COUNTS      1000    // ~2 seconds at 2ms/count
#define DISPLAY_UPDATE_TICKS    50      // Update display every 50 refreshes
#define VDD_REFRESH_CYCLES      10      // Refresh VDD every 10 main loops (~80s)
#define LABEL_DISPLAY_MS        1000    // Show label for 1 second
#define VOLTAGE_DISPLAY_MS      3000    // Show voltage for 3 seconds

// ADC constants
#define ADC_SAMPLES             16      // Number of samples to average
#define ADC_TIMEOUT             10000   // ADC conversion timeout
#define ADC_ERROR_VALUE         0xFFFF  // Sentinel for ADC errors

// Voltage calculation (fixed-point Q16.16 scaled)
#define VOLTAGE_MULTIPLIER      125     // Voltage divider ratio
#define ADC_RESOLUTION          1024    // 10-bit ADC
#define VREF_INTERNAL_MV        1200    // Internal reference = 1.2V = 1200mV
#define VDD_MIN_MV              4000    // Minimum valid VDD
#define VDD_MAX_MV              6000    // Maximum valid VDD
#define VDD_DEFAULT_MV          5000    // Default VDD if measurement fails

// Calibration
#define MAX_CALIBRATION         99
#define FLASH_CAL_PAGE_BASE     ((uint32_t)0x00003C00)
#define FLASH_CAL_ADDR          (FLASH_CAL_PAGE_BASE)
#define FLASH_CAL_MAGIC_ADDR    (FLASH_CAL_PAGE_BASE + 4)
#define FLASH_CAL_MAGIC         ((uint32_t)0xCAFEBABE)

// Kalman filter constants (fixed-point scaled by 1000)
#define KALMAN_P_INIT           100000  // Initial uncertainty (100.0)
#define KALMAN_P_RESET          10000   // Reset uncertainty (10.0)
#define KALMAN_P_MIN            1       // Minimum P (0.001)
#define KALMAN_Q_STABLE         10      // Process noise stable (0.01)
#define KALMAN_Q_MEDIUM         50      // Process noise medium (0.05)
#define KALMAN_Q_FAST           200     // Process noise fast (0.2)
#define KALMAN_R                500     // Measurement noise (0.5)
#define KALMAN_INNOVATION_HIGH  20000   // 20.0 threshold
#define KALMAN_INNOVATION_MED   5000    // 5.0 threshold

// Display update thresholds
#define DIFF_JUMP_THRESHOLD     5       // Jump directly if diff >= 5
#define DIFF_LARGE_THRESHOLD    10      // Large change threshold
#define STABLE_COUNT_FAST       1       // Counts needed for fast update
#define STABLE_COUNT_SLOW       3       // Counts needed for slow update
#define ZERO_THRESHOLD          2       // Consider as zero if <= 2
#define ZERO_JUMP_THRESHOLD     10      // Jump to zero if display > 10

// Watchdog
#define IWDG_PRESCALER          IWDG_Prescaler_128
#define IWDG_RELOAD_VALUE       0xFFF   // ~1.6 seconds timeout

/*===================================================================================
 * 7-Segment Patterns (Common Anode - active LOW segments)
 *===================================================================================*/
static const uint8_t DIGIT_PAT[10] = {
    0x3F, // 0: ABCDEF
    0x06, // 1: BC
    0x5B, // 2: ABDEG
    0x4F, // 3: ABCDG
    0x66, // 4: BCFG
    0x6D, // 5: ACDFG
    0x7D, // 6: ACDEFG
    0x07, // 7: ABC
    0x7F, // 8: ABCDEFG
    0x6F  // 9: ABCDFG
};

// Special display patterns
#define PATTERN_BLANK   0x00    // All segments off
#define PATTERN_DASH    0x40    // G only (dash)
#define PATTERN_O       0x3F    // ABCDEF (like 0)
#define PATTERN_P       0x73    // ABEFG
#define PATTERN_I       0x06    // BC (like 1)
#define PATTERN_E       0x79    // ADEFG
#define PATTERN_r       0x50    // EG
#define PATTERN_L       0x38    // DEF
#define PATTERN_H       0x76    // BCEFG

/*===================================================================================
 * System State Flags
 *===================================================================================*/
typedef enum {
    STATE_NORMAL = 0,
    STATE_ERROR_ADC,
    STATE_OVERLOAD,
    STATE_CALIBRATING
} SystemState;

static SystemState system_state = STATE_NORMAL;

/*===================================================================================
 * Global Variables
 *===================================================================================*/
static uint8_t segPattern[3] = {0, 0, 0};
static int16_t cal_offset_out = 0;
static int16_t cal_offset_in = 0;
static uint8_t inc_stable = 1, inc_cnt = 0;
static uint8_t dec_stable = 1, dec_cnt = 0;

// VDD in millivolts (fixed-point, no float)
static uint16_t actual_vdd_mv = VDD_DEFAULT_MV;

// ADC error tracking
static uint8_t adc_error_count = 0;
#define ADC_ERROR_THRESHOLD 5  // Show error after 5 consecutive failures

/*===================================================================================
 * Fixed-Point Kalman Filter State
 * Using Q16.16 format where values are scaled by 1000
 *===================================================================================*/
typedef struct {
    int32_t x;          // Estimated voltage (scaled by 1000)
    int32_t p;          // Estimation error covariance (scaled by 1000)
    int32_t q;          // Process noise covariance (scaled by 1000)
    int32_t r;          // Measurement noise covariance (scaled by 1000)
    int32_t k;          // Kalman gain (scaled by 1000)
    uint8_t initialized;
} KalmanState;

static KalmanState kalman_out;
static KalmanState kalman_in;

// Display state
static uint16_t display_out = 0;
static uint16_t display_in = 0;
static uint8_t stable_count_out = 0;
static uint8_t stable_count_in = 0;

// Calibration tracking
static int16_t last_saved_out = 0;
static int16_t last_saved_in = 0;

/*===================================================================================
 * Function Prototypes
 *===================================================================================*/
void GPIO_Init_All(void);
void ADC_Init_Custom(void);
uint16_t ADC_Read(uint8_t channel);
void Measure_VDD(void);
void Kalman_Init(KalmanState *ks);
int32_t Kalman_Update(KalmanState *ks, int32_t measurement);
uint16_t ReadVoltage_Kalman(uint8_t channel);
void SetSegment(uint8_t pattern);
void UpdateDisplay_Value(uint16_t value);
void UpdateDisplay_Label(char first, char second);
void UpdateDisplay_Error(void);
void UpdateDisplay_Overload(void);
void Display_Refresh(uint8_t *digit_idx);
void DelayWithDisplay(uint16_t ms);
void Buttons_Update(uint8_t *inc_press, uint8_t *dec_press);
void Save_Calibration(void);
void Save_Calibration_If_Changed(void);
void Load_Calibration(void);
uint16_t ApplyOffset(uint16_t v, int16_t offset);
void ShowVoltageWithCalibration(uint8_t channel, uint16_t normal_ms, int16_t *offset_ptr);
void IWDG_Init(void);
void IWDG_Feed(void);

/*===================================================================================
 * Watchdog Timer Functions
 *===================================================================================*/
void IWDG_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_PRESCALER);
    IWDG_SetReload(IWDG_RELOAD_VALUE);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}

/*===================================================================================
 * GPIO Initialization
 *===================================================================================*/
void GPIO_Init_All(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable peripheral clocks
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOC |
        RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_ADC1,
        ENABLE
    );

    // Buttons as input with pull-up
    GPIO_InitStructure.GPIO_Pin = BTN_INC_PIN | BTN_DEC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 7-Segment outputs - Push-pull, high speed
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // Port C segments
    GPIO_InitStructure.GPIO_Pin = SEG_C_PIN | SEG_D_PIN | SEG_G_PIN | SEG_E_PIN | SEG_F_PIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Port D segments
    GPIO_InitStructure.GPIO_Pin = SEG_A_PIN | SEG_B_PIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Digit select pins on Port D
    GPIO_InitStructure.GPIO_Pin = DIGIT2_PIN | DIGIT3_PIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Digit 1 on Port C
    GPIO_InitStructure.GPIO_Pin = DIGIT1_PIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ADC pins as analog input
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  // PC4
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  // PD4
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Initialize all segments OFF (HIGH for common anode)
    GPIO_SetBits(GPIOD, SEG_A_PIN | SEG_B_PIN);
    GPIO_SetBits(GPIOC, SEG_C_PIN | SEG_D_PIN | SEG_G_PIN | SEG_E_PIN | SEG_F_PIN);

    // Initialize all digits OFF (LOW for common anode)
    GPIO_ResetBits(DIGIT1_PORT, DIGIT1_PIN);
    GPIO_ResetBits(DIGIT2_PORT, DIGIT2_PIN);
    GPIO_ResetBits(DIGIT3_PORT, DIGIT3_PIN);
}

/*===================================================================================
 * ADC Initialization
 *===================================================================================*/
void ADC_Init_Custom(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};

    RCC_ADCCLKConfig(RCC_PCLK2_Div8);  // ADC clock = 24MHz/8 = 3MHz

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    // ADC self-calibration
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

/*===================================================================================
 * ADC Read with Error Detection
 *===================================================================================*/
uint16_t ADC_Read(uint8_t channel)
{
    uint16_t timeout = ADC_TIMEOUT;

    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_241Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        if(--timeout == 0) {
            adc_error_count++;
            if(adc_error_count >= ADC_ERROR_THRESHOLD) {
                system_state = STATE_ERROR_ADC;
            }
            return ADC_ERROR_VALUE;
        }
    }

    adc_error_count = 0;  // Reset on successful read
    if(system_state == STATE_ERROR_ADC) {
        system_state = STATE_NORMAL;
    }
    return ADC_GetConversionValue(ADC1);
}

/*===================================================================================
 * VDD Measurement using Internal Reference (Fixed-Point)
 *===================================================================================*/
void Measure_VDD(void)
{
    uint32_t sum = 0;
    uint8_t i;
    uint8_t valid_samples = 0;

    Delay_Ms(10);

    for(i = 0; i < ADC_SAMPLES; i++) {
        uint16_t sample = ADC_Read(ADC_CH_VREF);
        if(sample != ADC_ERROR_VALUE && sample > 0) {
            sum += sample;
            valid_samples++;
        }
        Delay_Us(100);
    }

    if(valid_samples > 0) {
        uint16_t vref_raw = sum / valid_samples;

        // VDD = (Vref * ADC_MAX) / ADC_reading
        // VDD_mv = (1200 * 1024) / vref_raw
        if(vref_raw > 100 && vref_raw < 400) {
            uint32_t vdd_calc = (uint32_t)VREF_INTERNAL_MV * ADC_RESOLUTION / vref_raw;

            if(vdd_calc >= VDD_MIN_MV && vdd_calc <= VDD_MAX_MV) {
                actual_vdd_mv = (uint16_t)vdd_calc;
            } else {
                actual_vdd_mv = VDD_DEFAULT_MV;
            }
        } else {
            actual_vdd_mv = VDD_DEFAULT_MV;
        }
    } else {
        actual_vdd_mv = VDD_DEFAULT_MV;
    }
}

/*===================================================================================
 * Fixed-Point Kalman Filter Implementation
 * All values scaled by 1000 for precision without floats
 *===================================================================================*/
void Kalman_Init(KalmanState *ks)
{
    ks->x = 0;
    ks->p = KALMAN_P_INIT;
    ks->q = KALMAN_Q_STABLE;
    ks->r = KALMAN_R;
    ks->k = 0;
    ks->initialized = 0;
}

int32_t Kalman_Update(KalmanState *ks, int32_t measurement)
{
    if(!ks->initialized) {
        ks->x = measurement;
        ks->p = KALMAN_P_RESET;
        ks->initialized = 1;
        return measurement;
    }

    // Prediction: p_pred = p + q
    int32_t p_pred = ks->p + ks->q;

    // Kalman gain: k = p_pred / (p_pred + r)
    // Scale by 1000 for fixed-point division
    ks->k = (p_pred * 1000) / (p_pred + ks->r);

    // Update estimate: x = x + k * (measurement - x) / 1000
    int32_t innovation = measurement - ks->x;
    ks->x = ks->x + (ks->k * innovation) / 1000;

    // Update covariance: p = (1 - k/1000) * p_pred
    ks->p = ((1000 - ks->k) * p_pred) / 1000;

    // Prevent numeric instability
    if(ks->p < KALMAN_P_MIN) ks->p = KALMAN_P_MIN;

    // Adaptive process noise based on innovation magnitude
    int32_t abs_innovation = (innovation < 0) ? -innovation : innovation;

    if(abs_innovation > KALMAN_INNOVATION_HIGH) {
        ks->q = KALMAN_Q_FAST;
    } else if(abs_innovation > KALMAN_INNOVATION_MED) {
        ks->q = KALMAN_Q_MEDIUM;
    } else {
        ks->q = KALMAN_Q_STABLE;
    }

    return ks->x;
}

/*===================================================================================
 * Voltage Measurement with Kalman Filter (Fixed-Point)
 *===================================================================================*/
uint16_t ReadVoltage_Kalman(uint8_t channel)
{
    uint32_t sum = 0;
    uint8_t i;
    uint8_t valid_samples = 0;
    uint16_t adc_avg;
    int32_t voltage_raw, voltage_filtered;
    uint16_t voltage_int;
    int16_t diff;

    KalmanState *ks;
    uint16_t *display;
    uint8_t *stable_count;
    uint8_t adc_channel;

    // Select channel
    if(channel == 0) {
        ks = &kalman_out;
        display = &display_out;
        stable_count = &stable_count_out;
        adc_channel = ADC_CH_OUTPUT;
    } else {
        ks = &kalman_in;
        display = &display_in;
        stable_count = &stable_count_in;
        adc_channel = ADC_CH_INPUT;
    }

    // Average ADC samples for noise reduction
    for(i = 0; i < ADC_SAMPLES; i++) {
        uint16_t sample = ADC_Read(adc_channel);
        if(sample != ADC_ERROR_VALUE) {
            sum += sample;
            valid_samples++;
        }
    }

    // Handle all samples failed
    if(valid_samples > 0) {
        adc_avg = sum / valid_samples;
    } else {
        adc_avg = 0;
    }

    // Fixed-point voltage calculation (scaled by 1000)
    // voltage = adc * vdd_mv * MULTIPLIER / (ADC_RESOLUTION * 1000)
    // Rearranged to avoid overflow: (adc * vdd_mv / ADC_RESOLUTION) * MULTIPLIER / 1000
    voltage_raw = ((int32_t)adc_avg * actual_vdd_mv / ADC_RESOLUTION) * VOLTAGE_MULTIPLIER;

    // Apply Kalman filter
    voltage_filtered = Kalman_Update(ks, voltage_raw);

    // Convert to display integer (divide by 1000 with rounding)
    voltage_int = (uint16_t)((voltage_filtered + 500) / 1000);

    // Check for overload
    if(voltage_int > OVERLOAD_THRESHOLD) {
        system_state = STATE_OVERLOAD;
        voltage_int = MAX_DISPLAY_VALUE;
    } else if(system_state == STATE_OVERLOAD) {
        system_state = STATE_NORMAL;
    }

    if(voltage_int > MAX_DISPLAY_VALUE) voltage_int = MAX_DISPLAY_VALUE;

    // Handle first reading initialization
    if(!ks->initialized || (*display == 0 && voltage_int == 0)) {
        *display = voltage_int;
        *stable_count = 0;
        return *display;
    }

    diff = (int16_t)voltage_int - (int16_t)(*display);
    if(diff < 0) diff = -diff;

    // Fast jump to 0 when input is disconnected
    if(voltage_int <= ZERO_THRESHOLD && *display > ZERO_JUMP_THRESHOLD) {
        *display = 0;
        *stable_count = 0;
        return *display;
    }

    // Update logic based on Kalman gain (confidence)
    if(ks->k > 500) {  // k > 0.5 (scaled by 1000)
        // High Kalman gain = big change detected, update fast
        if(diff >= DIFF_JUMP_THRESHOLD) {
            *display = voltage_int;
            *stable_count = 0;
        } else if(diff >= 1) {
            if(*stable_count >= STABLE_COUNT_FAST) {
                if(voltage_int > *display) (*display)++;
                else if(*display > 0) (*display)--;
                *stable_count = 0;
            } else {
                if(*stable_count < 255) (*stable_count)++;
            }
        }
    } else {
        // Low Kalman gain = stable, require more confirmation
        if(diff >= DIFF_LARGE_THRESHOLD) {
            *display = voltage_int;
            *stable_count = 0;
        } else if(diff >= 1) {
            if(*stable_count >= STABLE_COUNT_SLOW) {
                if(voltage_int > *display) (*display)++;
                else if(*display > 0) (*display)--;
                *stable_count = 0;
            } else {
                if(*stable_count < 255) (*stable_count)++;
            }
        } else {
            *stable_count = 0;
        }
    }

    return *display;
}

/*===================================================================================
 * 7-Segment Display Functions
 *===================================================================================*/
void SetSegment(uint8_t pattern)
{
    // For common anode: LOW = segment ON, HIGH = segment OFF
    GPIO_WriteBit(SEG_A_PORT, SEG_A_PIN, (pattern & 0x01) ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(SEG_B_PORT, SEG_B_PIN, (pattern & 0x02) ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(SEG_C_PORT, SEG_C_PIN, (pattern & 0x04) ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(SEG_D_PORT, SEG_D_PIN, (pattern & 0x08) ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(SEG_E_PORT, SEG_E_PIN, (pattern & 0x10) ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(SEG_F_PORT, SEG_F_PIN, (pattern & 0x20) ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(SEG_G_PORT, SEG_G_PIN, (pattern & 0x40) ? Bit_RESET : Bit_SET);
}

void UpdateDisplay_Value(uint16_t value)
{
    uint16_t h = value / 100;
    uint16_t t = (value / 10) % 10;
    uint16_t o = value % 10;

    // Leading zero suppression
    segPattern[0] = (h == 0) ? PATTERN_BLANK : DIGIT_PAT[h];
    segPattern[1] = (h == 0 && t == 0) ? PATTERN_BLANK : DIGIT_PAT[t];
    segPattern[2] = DIGIT_PAT[o];
}

void UpdateDisplay_Label(char first, char second)
{
    segPattern[0] = PATTERN_DASH;
    segPattern[1] = (first == 'O') ? PATTERN_O : ((first == 'I') ? PATTERN_I : PATTERN_BLANK);
    segPattern[2] = (second == 'P') ? PATTERN_P : PATTERN_BLANK;
}

void UpdateDisplay_Error(void)
{
    // Display "Err"
    segPattern[0] = PATTERN_E;
    segPattern[1] = PATTERN_r;
    segPattern[2] = PATTERN_r;
}

void UpdateDisplay_Overload(void)
{
    // Display "OL" (Overload)
    segPattern[0] = PATTERN_BLANK;
    segPattern[1] = PATTERN_O;
    segPattern[2] = PATTERN_L;
}

void Display_Refresh(uint8_t *digit_idx)
{
    // Turn off all digits (LOW = OFF for common anode)
    GPIO_ResetBits(DIGIT1_PORT, DIGIT1_PIN);
    GPIO_ResetBits(DIGIT2_PORT, DIGIT2_PIN);
    GPIO_ResetBits(DIGIT3_PORT, DIGIT3_PIN);

    // Set segment pattern
    SetSegment(segPattern[*digit_idx]);

    // Turn on current digit (HIGH = ON for common anode)
    if(*digit_idx == 0) GPIO_SetBits(DIGIT1_PORT, DIGIT1_PIN);
    else if(*digit_idx == 1) GPIO_SetBits(DIGIT2_PORT, DIGIT2_PIN);
    else GPIO_SetBits(DIGIT3_PORT, DIGIT3_PIN);

    Delay_Us(2000);

    (*digit_idx)++;
    if(*digit_idx > 2) *digit_idx = 0;
}

void DelayWithDisplay(uint16_t ms)
{
    uint8_t d = 0;
    uint16_t cycles = (ms + 5) / 6;  // ~6ms per full 3-digit cycle
    uint16_t i;

    if(cycles == 0) cycles = 1;

    for(i = 0; i < cycles; i++) {
        IWDG_Feed();  // Feed watchdog during delays
        Display_Refresh(&d);
        Display_Refresh(&d);
        Display_Refresh(&d);
    }
}

/*===================================================================================
 * Button Handling with Debounce
 *===================================================================================*/
void Buttons_Update(uint8_t *inc_press, uint8_t *dec_press)
{
    uint8_t raw;
    *inc_press = 0;
    *dec_press = 0;

    // Increment button
    raw = GPIO_ReadInputDataBit(BTN_INC_PORT, BTN_INC_PIN);
    if(raw != inc_stable) {
        if(++inc_cnt >= DEBOUNCE_COUNTS) {
            inc_stable = raw;
            inc_cnt = 0;
            if(inc_stable == 0) *inc_press = 1;  // Active low
        }
    } else {
        inc_cnt = 0;
    }

    // Decrement button
    raw = GPIO_ReadInputDataBit(BTN_DEC_PORT, BTN_DEC_PIN);
    if(raw != dec_stable) {
        if(++dec_cnt >= DEBOUNCE_COUNTS) {
            dec_stable = raw;
            dec_cnt = 0;
            if(dec_stable == 0) *dec_press = 1;  // Active low
        }
    } else {
        dec_cnt = 0;
    }
}

/*===================================================================================
 * Flash Storage for Calibration
 *===================================================================================*/
void Save_Calibration(void)
{
    uint32_t word_offsets;
    uint32_t word_magic = FLASH_CAL_MAGIC;
    FLASH_Status status;

    // Pack calibration values into one word
    uint16_t out_u = (uint16_t)cal_offset_out;
    uint16_t in_u = (uint16_t)cal_offset_in;
    word_offsets = ((uint32_t)in_u << 16) | (uint32_t)out_u;

    // Check if flash already contains correct values (avoid unnecessary writes)
    uint32_t cur_offsets = *(__IO uint32_t *)FLASH_CAL_ADDR;
    uint32_t cur_magic = *(__IO uint32_t *)FLASH_CAL_MAGIC_ADDR;

    if(cur_offsets == word_offsets && cur_magic == word_magic) {
        return;  // No change needed
    }

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_ERROR_PG | FLASH_FLAG_WRPRTERR);

    status = FLASH_ErasePage(FLASH_CAL_PAGE_BASE);
    if(status != FLASH_COMPLETE) {
        FLASH_Lock();
        return;
    }

    status = FLASH_ProgramWord(FLASH_CAL_ADDR, word_offsets);
    if(status != FLASH_COMPLETE) {
        FLASH_Lock();
        return;
    }

    status = FLASH_ProgramWord(FLASH_CAL_MAGIC_ADDR, word_magic);
    if(status != FLASH_COMPLETE) {
        FLASH_Lock();
        return;
    }

    FLASH_Lock();
}

void Save_Calibration_If_Changed(void)
{
    if(cal_offset_out != last_saved_out || cal_offset_in != last_saved_in) {
        Save_Calibration();
        last_saved_out = cal_offset_out;
        last_saved_in = cal_offset_in;
    }
}

void Load_Calibration(void)
{
    uint32_t stored_offsets = *(__IO uint32_t *)FLASH_CAL_ADDR;
    uint32_t magic = *(__IO uint32_t *)FLASH_CAL_MAGIC_ADDR;

    if(magic == FLASH_CAL_MAGIC && stored_offsets != 0xFFFFFFFFu) {
        int16_t out = (int16_t)(stored_offsets & 0xFFFFu);
        int16_t in = (int16_t)((stored_offsets >> 16) & 0xFFFFu);

        // Clamp to valid range
        if(out > MAX_CALIBRATION) out = MAX_CALIBRATION;
        if(out < -MAX_CALIBRATION) out = -MAX_CALIBRATION;
        if(in > MAX_CALIBRATION) in = MAX_CALIBRATION;
        if(in < -MAX_CALIBRATION) in = -MAX_CALIBRATION;

        cal_offset_out = out;
        cal_offset_in = in;
    } else {
        cal_offset_out = 0;
        cal_offset_in = 0;
    }

    last_saved_out = cal_offset_out;
    last_saved_in = cal_offset_in;
}

/*===================================================================================
 * Calibration Helper Functions
 *===================================================================================*/
uint16_t ApplyOffset(uint16_t v, int16_t offset)
{
    int16_t adj = (int16_t)v + offset;
    if(adj < 0) adj = 0;
    if(adj > MAX_DISPLAY_VALUE) adj = MAX_DISPLAY_VALUE;
    return (uint16_t)adj;
}

void ShowVoltageWithCalibration(uint8_t channel, uint16_t normal_ms, int16_t *offset_ptr)
{
    uint16_t v;
    uint8_t digit = 0;
    uint16_t update_tick = 0;
    uint8_t inc_evt, dec_evt;
    uint8_t cal_mode = 0;
    uint16_t idle_count = 0;

    // Prime both Kalman filters
    ReadVoltage_Kalman(0);
    ReadVoltage_Kalman(1);

    v = ReadVoltage_Kalman(channel);
    v = ApplyOffset(v, *offset_ptr);

    // Initial display update based on state
    if(system_state == STATE_ERROR_ADC) {
        UpdateDisplay_Error();
    } else if(system_state == STATE_OVERLOAD) {
        UpdateDisplay_Overload();
    } else {
        UpdateDisplay_Value(v);
    }

    while(1) {
        IWDG_Feed();  // Feed watchdog in main loop
        Display_Refresh(&digit);

        if(++update_tick >= DISPLAY_UPDATE_TICKS) {
            update_tick = 0;

            uint16_t v_out = ReadVoltage_Kalman(0);
            uint16_t v_in = ReadVoltage_Kalman(1);

            if(channel == 0) {
                v = ApplyOffset(v_out, *offset_ptr);
            } else {
                v = ApplyOffset(v_in, *offset_ptr);
            }

            // Update display based on current state
            if(system_state == STATE_ERROR_ADC) {
                UpdateDisplay_Error();
            } else if(system_state == STATE_OVERLOAD) {
                UpdateDisplay_Overload();
            } else {
                UpdateDisplay_Value(v);
            }
        }

        Buttons_Update(&inc_evt, &dec_evt);

        if(inc_evt) {
            (*offset_ptr)++;
            if(*offset_ptr > MAX_CALIBRATION) *offset_ptr = MAX_CALIBRATION;
            cal_mode = 1;
            idle_count = CAL_TIMEOUT_COUNTS;
            update_tick = DISPLAY_UPDATE_TICKS;  // Force immediate update
        }

        if(dec_evt) {
            (*offset_ptr)--;
            if(*offset_ptr < -MAX_CALIBRATION) *offset_ptr = -MAX_CALIBRATION;
            cal_mode = 1;
            idle_count = CAL_TIMEOUT_COUNTS;
            update_tick = DISPLAY_UPDATE_TICKS;  // Force immediate update
        }

        if(cal_mode) {
            if(idle_count) {
                idle_count--;
            } else {
                Save_Calibration_If_Changed();
                break;
            }
        } else {
            if(normal_ms) {
                normal_ms--;
            } else {
                break;
            }
        }
    }
}

/*===================================================================================
 * Main Function
 *===================================================================================*/
int main(void)
{
    uint8_t vdd_refresh_counter = 0;

    // System initialization
    SystemInit();
    Delay_Init();
    GPIO_Init_All();
    ADC_Init_Custom();
    Measure_VDD();

    // Initialize Kalman filters
    Kalman_Init(&kalman_out);
    Kalman_Init(&kalman_in);

    // Load saved calibration from flash
    Load_Calibration();

    // Initialize watchdog (1.6 second timeout)
    IWDG_Init();

    Delay_Ms(100);

    // Main loop
    while(1) {
        IWDG_Feed();

        // Periodic VDD measurement to compensate for supply drift
        if(++vdd_refresh_counter >= VDD_REFRESH_CYCLES) {
            vdd_refresh_counter = 0;
            Measure_VDD();
        }

        // Display Output voltage
        UpdateDisplay_Label('O', 'P');
        DelayWithDisplay(LABEL_DISPLAY_MS);

        ShowVoltageWithCalibration(0, VOLTAGE_DISPLAY_MS, &cal_offset_out);

        // Display Input voltage
        UpdateDisplay_Label('I', 'P');
        DelayWithDisplay(LABEL_DISPLAY_MS);

        ShowVoltageWithCalibration(1, VOLTAGE_DISPLAY_MS, &cal_offset_in);
    }

    return 0;
}
