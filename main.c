
/*===================================================================================
 * Professional Voltmeter with Kalman Filter
 * For CH32V003A4M6 (RISC-V)
 * Matches Aashi Technologies Performance
 *===================================================================================*/
#include "debug.h"
#include "ch32v00x_conf.h"
#include <stdint.h>
#include "ch32v00x_flash.h"
/*===================================================================================
 * Hardware Pin Configuration
 *===================================================================================*/
// Buttons
#define BTN_INC_PORT    GPIOA
#define BTN_INC_PIN     GPIO_Pin_1
#define BTN_DEC_PORT    GPIOA
#define BTN_DEC_PIN     GPIO_Pin_2

// ADC Channels
#define ADC_CH_OUTPUT   ADC_Channel_7  // PD4
#define ADC_CH_INPUT    ADC_Channel_2  // PC4

// 7-Segment pins (A-G)
#define SEG_A_PORT      GPIOD
#define SEG_A_PIN       GPIO_Pin_1
#define SEG_B_PORT      GPIOD
#define SEG_B_PIN       GPIO_Pin_6
#define SEG_C_PORT      GPIOC
#define SEG_C_PIN       GPIO_Pin_2
#define SEG_D_PORT      GPIOC
#define SEG_D_PIN       GPIO_Pin_1
#define SEG_E_PORT      GPIOC
#define SEG_E_PIN       GPIO_Pin_3
#define SEG_F_PORT      GPIOC
#define SEG_F_PIN       GPIO_Pin_6
#define SEG_G_PORT      GPIOC
#define SEG_G_PIN       GPIO_Pin_0

// Digit select
#define DIGIT1_PORT     GPIOC
#define DIGIT1_PIN      GPIO_Pin_7
#define DIGIT2_PORT     GPIOD
#define DIGIT2_PIN      GPIO_Pin_5
#define DIGIT3_PORT     GPIOD
#define DIGIT3_PIN      GPIO_Pin_7

/*===================================================================================
 * Configuration
 *===================================================================================*/
#define MAX_DISPLAY_VALUE     999
#define DEBOUNCE_MS           20
#define MAX_CALIBRATION       99
#define CAL_TIMEOUT_MS        2000

#define VOLTAGE_MULTIPLIER_NUM    125
#define VOLTAGE_MULTIPLIER_DENOM  1

/* Last 1K page reserved for calibration:
 * FLASH page size = 1K, total 16K -> pages 0..15
 * Page 15 base = 0x00003C00
 */
#define FLASH_CAL_PAGE_BASE   ((uint32_t)0x00003C00)

/* We��ll store 2 words:
 * [0] = cal_out (low 16 bits) + cal_in (high 16 bits)
 * [1] = magic number for validity check
 */
#define FLASH_CAL_ADDR        (FLASH_CAL_PAGE_BASE)        // word0
#define FLASH_CAL_MAGIC_ADDR  (FLASH_CAL_PAGE_BASE + 4)    // word1
#define FLASH_CAL_MAGIC  ((uint32_t)0xCAFEBABE)  // More distinctive


/*===================================================================================
 * 7-Segment Patterns (Common anode)
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

#define PATTERN_O    0x3F
#define PATTERN_P    0x73
#define PATTERN_I    0x06
#define PATTERN_DASH 0x40

/*===================================================================================
 * Global Variables
 *===================================================================================*/
static uint8_t segPattern[3] = {0,0,0};
static int16_t cal_offset_out = 0;
static int16_t cal_offset_in  = 0;
static uint8_t inc_stable = 1, inc_cnt = 0;
static uint8_t dec_stable = 1, dec_cnt = 0;
static float actual_vdd= 5.0f;
/*===================================================================================
 * Kalman Filter State (per channel)
 *===================================================================================*/
typedef struct {
    float x;          // Estimated voltage
    float p;          // Estimation error covariance
    float q;          // Process noise covariance (how much we expect it to change)
    float r;          // Measurement noise covariance (sensor noise)
    float k;          // Kalman gain
    uint8_t initialized;
} KalmanState;

static KalmanState kalman_out;
static KalmanState kalman_in;

// Display state
static uint16_t display_out = 0;
static uint16_t display_in = 0;
static uint8_t stable_count_out = 0;
static uint8_t stable_count_in = 0;



/*===================================================================================
 * GPIO Initialization
 *===================================================================================*/
void GPIO_Init_All(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    

    // Enable peripheral clocks for GPIOA/C/D and ADC1
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
    
    // 7-Segment outputs
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // Port C segments
    GPIO_InitStructure.GPIO_Pin = SEG_C_PIN | SEG_D_PIN | SEG_G_PIN | SEG_E_PIN | SEG_F_PIN ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // Port D segments and digits
    GPIO_InitStructure.GPIO_Pin = SEG_A_PIN | SEG_B_PIN ;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // Port A segments and digit
    GPIO_InitStructure.GPIO_Pin =DIGIT2_PIN | DIGIT3_PIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // Digit 2
    GPIO_InitStructure.GPIO_Pin = DIGIT1_PIN ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    //ADC-
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    // ADC pins as analog input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  // PC4
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  // PD4
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // All segments off initially (HIGH = segment OFF for common anode)
    GPIO_SetBits(GPIOD, SEG_A_PIN | SEG_B_PIN);
    GPIO_SetBits(GPIOC, SEG_C_PIN | SEG_D_PIN | SEG_G_PIN | SEG_E_PIN | SEG_F_PIN);

    // All digits off initially (LOW = digit OFF for common anode)
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
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div8); // ADC clock = 24MHz/8 = 3MHz
    
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    ADC_Cmd(ADC1, ENABLE);
    
    // Calibration
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

uint16_t ADC_Read(uint8_t channel)
{
    uint16_t timeout = 10000;
    
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_241Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        if(--timeout == 0) {
            return 0;  // Timeout protection
        }
    }
    
    return ADC_GetConversionValue(ADC1);
}

void Measure_VDD(void)
{
    uint16_t vref_raw;
    uint8_t i;
    uint32_t sum = 0;
    
    Delay_Ms(10);
    
    for(i = 0; i < 16; i++) {
        sum += ADC_Read(ADC_Channel_8);  // Channel 8 is internal 1.2V ref
        Delay_Us(100);
    }
    vref_raw = sum >> 4;
    
    if(vref_raw > 100 && vref_raw < 400) {
        actual_vdd = (1.2f * 1024.0f) / (float)vref_raw;
        if(actual_vdd < 4.0f) actual_vdd = 5.0f;
        if(actual_vdd > 6.0f) actual_vdd = 5.0f;
    } else {
        actual_vdd = 5.0f;
    }
}


/*===================================================================================
 * Kalman Filter Implementation
 *===================================================================================*/
void Kalman_Init(KalmanState *ks)
{
    ks->x = 0.0f;
    ks->p = 100.0f;  // High initial uncertainty
    ks->q = 0.01f;    // Process noise (how much voltage can change per sample)
    ks->r = 0.5f;     // Measurement noise (ADC noise level)
    ks->k = 0.0f;
    ks->initialized = 0;
}


float Kalman_Update(KalmanState *ks, float measurement)
{
    if(!ks->initialized) {
        ks->x = measurement;
        ks->p = 10.0f;  // Reset uncertainty on first reading
        ks->initialized = 1;
        return measurement;
    }
    
    // Prediction
    float p_pred = ks->p + ks->q;
    
    // Update
    ks->k = p_pred / (p_pred + ks->r);
    ks->x = ks->x + ks->k * (measurement - ks->x);
    ks->p = (1.0f - ks->k) * p_pred;
    
    // Smoother adaptive noise based on innovation magnitude
    float innovation = measurement - ks->x;
    if(innovation < 0) innovation = -innovation;
    
    // Gradual adaptation instead of step change
    if(innovation > 20.0f) {
        ks->q = 0.2f;   // Large change - track faster
    } else if(innovation > 5.0f) {
        ks->q = 0.05f;  // Medium change
    } else {
        ks->q = 0.01f;  // Stable - smooth heavily
    }
    
    return ks->x;
}


/*===================================================================================
 * Voltage Measurement with Kalman Filter
 *===================================================================================*/
uint16_t ReadVoltage_Kalman(uint8_t channel)
{
    uint32_t sum = 0;
    uint8_t i;
    uint16_t adc_avg;
    float voltage_raw, voltage_filtered;
    uint16_t voltage_int;
    //uint16_t display_value;
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
    
    // Average 16 ADC samples for noise reduction
    for(i=0; i<16; i++) {
        sum += ADC_Read(adc_channel);
    }
    adc_avg = sum >> 4;
    
    voltage_raw = (float)adc_avg * actual_vdd / 1024.0f;
    voltage_raw = voltage_raw * VOLTAGE_MULTIPLIER_NUM / VOLTAGE_MULTIPLIER_DENOM;
    
    // Apply Kalman filter
    voltage_filtered = Kalman_Update(ks, voltage_raw);
    
    // Convert to integer
    voltage_int = (uint16_t)(voltage_filtered + 0.5f);
    
    if(voltage_int > MAX_DISPLAY_VALUE) voltage_int = MAX_DISPLAY_VALUE;
    
    // Smart display update logic
    if(*display == 0) {
        *display = voltage_int;
        *stable_count = 0;
        return *display;
    }
    
    diff = (int16_t)voltage_int - (int16_t)(*display);
    if(diff < 0) diff = -diff;
    
    // Update logic based on Kalman gain (confidence)
    if(ks->k > 0.5f) {
        // High Kalman gain = big change detected, update fast
        if(diff >= 5) {
            *display = voltage_int;
            *stable_count = 0;
        } else if(diff >= 1) {
            if(*stable_count >= 1) {
                if(voltage_int > *display) (*display)++;
                else (*display)--;
                *stable_count = 0;
            } else {
                (*stable_count)++;
            }
        }
    } else {
        // Low Kalman gain = stable, require more confirmation
        if(diff >= 10) {
            *display = voltage_int;
            *stable_count = 0;
        } else if(diff >= 1) {
            if(*stable_count >= 3) {
                if(voltage_int > *display) (*display)++;
                else (*display)--;
                *stable_count = 0;
            } else {
                (*stable_count)++;
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

    segPattern[0] = (h == 0) ? 0x00 : DIGIT_PAT[h];
    segPattern[1] = (h == 0 && t == 0) ? 0x00 : DIGIT_PAT[t];
    segPattern[2] = DIGIT_PAT[o];
}

void UpdateDisplay_Label(char first, char second)
{
    segPattern[0] = PATTERN_DASH;
    segPattern[1] = (first == 'O') ? PATTERN_O : ((first == 'I') ? PATTERN_I : 0x00);
    segPattern[2] = (second == 'P') ? PATTERN_P : 0x00;
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
    uint16_t cycles = (ms+5) / 6;  // Each refresh is ~2ms per digit �� 3 digits
    uint16_t i;

   if(cycles==0) cycles=1;
    for(i = 0; i < cycles; i++) {
        Display_Refresh(&d);
        Display_Refresh(&d);
        Display_Refresh(&d);  // Complete one full digit cycle
    }
}

/*===================================================================================
 * Button Handling
 *===================================================================================*/
void Buttons_Update(uint8_t *inc_press, uint8_t *dec_press)
{
    uint8_t raw;
    *inc_press = 0;
    *dec_press = 0;

    raw = GPIO_ReadInputDataBit(BTN_INC_PORT, BTN_INC_PIN);
    if(raw != inc_stable) {
        if(++inc_cnt >= DEBOUNCE_MS) {
            inc_stable = raw;
            inc_cnt = 0;
            if(inc_stable == 0) *inc_press = 1;
        }
    } else {
        inc_cnt = 0;
    }

    raw = GPIO_ReadInputDataBit(BTN_DEC_PORT, BTN_DEC_PIN);
    if(raw != dec_stable) {
        if(++dec_cnt >= DEBOUNCE_MS) {
            dec_stable = raw;
            dec_cnt = 0;
            if(dec_stable == 0) *dec_press = 1;
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
    /* Pack two int16_t into one 32-bit word:
     * low 16 bits  = cal_offset_out
     * high 16 bits = cal_offset_in
     */
    uint16_t out_u = (uint16_t)cal_offset_out;
    uint16_t in_u  = (uint16_t)cal_offset_in;
    word_offsets   = ((uint32_t)in_u << 16) | (uint32_t)out_u;

    /* If already the same in flash, don't wear the page again */
    uint32_t cur_offsets = *(__IO uint32_t *)FLASH_CAL_ADDR;
    uint32_t cur_magic   = *(__IO uint32_t *)FLASH_CAL_MAGIC_ADDR;

    if (cur_offsets == word_offsets && cur_magic == word_magic)
    {
        return; // nothing to do
    }

    /* --- Flash programming sequence for CH32V003 --- */
    
    FLASH_Unlock();  // unlock flash controller

    /* Clear any pending flags */
    FLASH_ClearFlag(FLASH_FLAG_BSY  |
                    FLASH_FLAG_EOP  |
                    FLASH_ERROR_PG  |
                    FLASH_FLAG_WRPRTERR);

    /* Erase the 1K page containing our data */
    status = FLASH_ErasePage(FLASH_CAL_PAGE_BASE);  // �� CHECK RETURN VALUE
    if(status != FLASH_COMPLETE) {
        FLASH_Lock();
        // Optional: Show error on display
        // UpdateDisplay_Label('E', 'r');
        return;
    }

    /* Program the two 32-bit words */
    status = FLASH_ProgramWord(FLASH_CAL_ADDR, word_offsets);  // �� CHECK THIS
    if(status != FLASH_COMPLETE) {
        FLASH_Lock();
        return;
    }

    status = FLASH_ProgramWord(FLASH_CAL_MAGIC_ADDR, word_magic);  // �� AND THIS
    if(status != FLASH_COMPLETE) {
        FLASH_Lock();
        return;
    }

    FLASH_Lock();
}
// In ShowVoltageWithCalibration, track if calibration actually changed
static int16_t last_saved_out = 0;
static int16_t last_saved_in = 0;

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
    uint32_t magic          = *(__IO uint32_t *)FLASH_CAL_MAGIC_ADDR;

    if (magic == FLASH_CAL_MAGIC && stored_offsets != 0xFFFFFFFFu)
    {
        int16_t out = (int16_t)( stored_offsets        & 0xFFFFu);
        int16_t in  = (int16_t)((stored_offsets >> 16) & 0xFFFFu);

        /* Clamp to allowed range just in case */
        if (out >  MAX_CALIBRATION) out =  MAX_CALIBRATION;
        if (out < -MAX_CALIBRATION) out = -MAX_CALIBRATION;
        if (in  >  MAX_CALIBRATION) in  =  MAX_CALIBRATION;
        if (in  < -MAX_CALIBRATION) in  = -MAX_CALIBRATION;

        cal_offset_out = out;
        cal_offset_in  = in;
    }
    else
    {
        /* Flash is blank or invalid */
        cal_offset_out = 0;
        cal_offset_in  = 0;
    }
 // After loading, sync the "last saved" trackers
    last_saved_out = cal_offset_out;
    last_saved_in = cal_offset_in;

}

/*===================================================================================
 * Main Calibration Display Function
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
    uint16_t idle_ms = 0;

    ReadVoltage_Kalman(0);
    ReadVoltage_Kalman(1);
    
    v = ReadVoltage_Kalman(channel);
    v = ApplyOffset(v, *offset_ptr);
    UpdateDisplay_Value(v);

    while(1) {
        Display_Refresh(&digit);

        if(++update_tick >= 50) {  // Faster update rate
            update_tick = 0;
            
            uint16_t v_out = ReadVoltage_Kalman(0);
            uint16_t v_in = ReadVoltage_Kalman(1);
            
            if(channel == 0) {
                v = ApplyOffset(v_out, *offset_ptr);
            } else {
                v = ApplyOffset(v_in, *offset_ptr);
            }
            UpdateDisplay_Value(v);
        }

        Buttons_Update(&inc_evt, &dec_evt);

        if(inc_evt) {
            (*offset_ptr)++;
            if(*offset_ptr > MAX_CALIBRATION) *offset_ptr = MAX_CALIBRATION;
            cal_mode = 1;
            idle_ms = CAL_TIMEOUT_MS;
            update_tick = 100;
        }

        if(dec_evt) {
            (*offset_ptr)--;
            if(*offset_ptr < -MAX_CALIBRATION) *offset_ptr = -MAX_CALIBRATION;
            cal_mode = 1;
            idle_ms = CAL_TIMEOUT_MS;
            update_tick = 100;
        }

        if(cal_mode) {
            if(idle_ms) idle_ms--;
            else {
                Save_Calibration_If_Changed();
                break;
            }
        } else {
            if(normal_ms) normal_ms--;
            else break;
        }
    }
}
/*void IWDG_Init(void) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_128);
    IWDG_SetReload(0xFFF);
    IWDG_ReloadCounter();
    IWDG_Enable();
}
*/

/*===================================================================================
 * Main Function
 *===================================================================================*/
int main(void)
{
    SystemInit();
    Delay_Init();
    GPIO_Init_All();
    ADC_Init_Custom();
    Measure_VDD();

    // Initialize Kalman filters
    Kalman_Init(&kalman_out);
    Kalman_Init(&kalman_in);

    Load_Calibration();

    Delay_Ms(100);

    while(1) {
        UpdateDisplay_Label('O', 'P');
        DelayWithDisplay(1000);

        ShowVoltageWithCalibration(0, 3000, &cal_offset_out);

        UpdateDisplay_Label('I', 'P');
        DelayWithDisplay(1000);

        ShowVoltageWithCalibration(1, 3000, &cal_offset_in);
    }

    return 0;
}