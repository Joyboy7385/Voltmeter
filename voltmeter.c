
/*#include "ms51_16k_keil.h"               // Base SFR register definitions */
#include "SFR_Macro_MS51_16K_keil.h"     // Low-level SFR macros (set/clear bits)
//#include "Function_define_MS51_16K_keil.h" // I/O and functional macros
//#include "Delay_keil.h"                  // Delay function declarations

bit BIT_TMP;   // definition for the macros in Function_define_MS51_16K_keil.h

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

/* Common–anode: drive segments LOW to light, so invert the 7 bits */
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
     O/P (–OP)  -> AIN0 = P1.7
     I/P (–IP)  -> AIN1 = P3.0

   Calibration buttons (active-LOW to GND with internal pull-ups):
     INC “+” -> P2.0
     DEC “–” -> P1.5
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
#define DEBOUNCE_MS     20
static unsigned char inc_stable = 1, inc_cnt = 0;
static unsigned char dec_stable = 1, dec_cnt = 0;

/* Helpers: update button debouncer once per ms, raise 1 on fresh press */
static void Buttons_Update(unsigned char *inc_press, unsigned char *dec_press)
{
    unsigned char raw;
    *inc_press = 0; *dec_press = 0;

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
static void ADC_InitChannels(void)
{
    /* Analog inputs */
    P17_Input_Mode; /* AIN0 */
    P30_Input_Mode; /* AIN1 */

    /* Disable digital input buffers on those pins (reduces leakage/noise) */
    AINDIDS |= SET_BIT0; /* P1.7 digital input disable */
    AINDIDS |= SET_BIT1; /* P3.0 digital input disable */

    /* Enable ADC module */
    ADCCON1 |= 0x01;

}

static unsigned int ADC_ReadChannel(unsigned char ch)
{
    unsigned int result;
    ADCCON0 &= 0xF0;
    ADCCON0 |= (ch & 0x0F);
    clr_ADCF;
    set_ADCS;
    while (ADCF == 0);
    result  = ADCRH;
    result  = (result << 4) | (ADCRL & 0x0F);
    return result; /* 12-bit: 0..4095 */
}
/* Measure actual VDD using internal 1.22V bandgap reference (ADCHS=8)
   Returns VDD in millivolts (e.g. 5000 for 5.00V)
*/
static unsigned int Measure_VDD_mV(void)
{
    unsigned long sum = 0;
    unsigned int adc_bg;
    unsigned char i;
	  unsigned int vdd_mV ;
    // Select bandgap channel (CH = 8)
    for (i=0; i<8; i++) {
        sum += ADC_ReadChannel(8);   // internal bandgap
    }
    adc_bg = (unsigned int)(sum / 8);

    // Avoid divide-by-zero
    if (adc_bg == 0)
			adc_bg = 1;

    // VDD = 1.22 * 4096 / ADC_BG
    vdd_mV = (1220UL * 4096UL) / adc_bg;

    return vdd_mV; // e.g. 4980 means 4.98V
}

static unsigned int ReadAverageVoltage(unsigned char channel, unsigned char samples)
{
    unsigned long sum = 0;
	  unsigned char i;
	  unsigned int avg;
	  unsigned int vdd_mV;
	  unsigned long vt;
	  unsigned int volts;
    for (i=0; i<samples; i++) {
        sum += ADC_ReadChannel(channel);
    }
    avg = (unsigned int)(sum / samples);

    // Measure supply voltage dynamically
    vdd_mV = Measure_VDD_mV();  // e.g. 5000 mV
    vt = (unsigned long)avg * vdd_mV * 125UL;
    // Divide by 4096 (12-bit ADC)
    volts = ((vt + 2048UL) / (4096UL * 1000UL)); // in volts

    if (volts > 999) 
			volts = 999; /* safety clamp for display */
   
		return volts;
}


/* ===== Display helpers ===== */
static void UpdateDisplayBufferForValue(unsigned int value)
{
    unsigned int h = value / 100U;
    unsigned int t = (value / 10U) % 10U;
    unsigned int o = value % 10U;

    segPattern[0] = (h==0) ? 0x00 : DIGIT_PAT[h];
    segPattern[1] = (h==0 && t==0) ? 0x00 : DIGIT_PAT[t];
    segPattern[2] = DIGIT_PAT[o];
}

static void UpdateDisplayBufferForLabel(char first, char second)
{
    segPattern[0] = PATTERN_DASH;
    segPattern[1] = (first=='O') ? PATTERN_O : ((first=='I') ? PATTERN_I : 0x00);
    segPattern[2] = (second=='P')? PATTERN_P : 0x00;
}

/* 1 ms multiplex refresh for whichever patterns are loaded */
static void Display_Refresh_1ms(unsigned char *digit_idx)
{
    /* Turn all off to avoid ghosting */
    DIGIT1=0; DIGIT2=0; DIGIT3=0;

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
    //Timer0_Delay1ms(1);
		Timer0_Delay(16000000UL, 1, 1000);   // if Fsys = 16 MHz

    /* Turn off current digit */
    if (*digit_idx == 0) 
			DIGIT1 = 0;
    else if (*digit_idx == 1) 
			DIGIT2 = 0;
    else 
			DIGIT3 = 0;

    (*digit_idx)++;
    if (*digit_idx > 2) 
			*digit_idx = 0;
}

/* Simple delay that keeps refreshing display (no buttons) */
static void DelayWithDisplay(unsigned int ms)
{
    unsigned char d = 0;
    while (ms--) {
        Display_Refresh_1ms(&d);
    }
}

/* Apply calibration offset and clamp to 0..999 */
static unsigned int ApplyOffset(unsigned int v, int offset)
{
    int adj = (int)v + offset;
    if (adj < 0) adj = 0;
    if (adj > 999) adj = 999;
    return (unsigned int)adj;
}

/* Show live voltage for channel with live calibration.
   - normal_ms: how long to show if user does NOT press buttons.
   - If user presses +/-: enter "cal mode", keep showing THIS channel;
     exit cal mode after 2000 ms of no further presses, then return.
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

    /* Initial reading and display */
    unsigned int v = ReadAverageVoltage(channel, 8);
    v = ApplyOffset(v, *offset_ptr);
    UpdateDisplayBufferForValue(v);

    while (1) {
        /* 1ms refresh */
        Display_Refresh_1ms(&digit);

        /* Update reading every 100ms for smoothness */
        if (++update_tick >= 100) {
            update_tick = 0;
            v = ReadAverageVoltage(channel, 8);
            v = ApplyOffset(v, *offset_ptr);
            UpdateDisplayBufferForValue(v);
        }

        /* Scan buttons once per ms */
        Buttons_Update(&inc_evt, &dec_evt);

        if (inc_evt) {
            (*offset_ptr)++;
            if (*offset_ptr > 99) *offset_ptr = 99;      /* limit ±99 V */
            cal_mode = 1; idle_ms = 2000; update_tick = 100; /* force display update */
        }
        if (dec_evt) {
            (*offset_ptr)--;
            if (*offset_ptr < -99) *offset_ptr = -99;
            cal_mode = 1; idle_ms = 2000; update_tick = 100;
        }

        /* Exit conditions */
        if (cal_mode) {
            if (idle_ms) 
							idle_ms--;
            else 
							break;                /* 2s no presses ? leave cal mode */
        } else {
            if (normal_ms) 
							normal_ms--;
            else 
							break;                /* finished normal display duration */
        }
    }
}

/* ===== Init I/O ===== */
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
    IO_Init();
    ADC_InitChannels();

    while (1) {
        /* 1) Label -OP for 1s */
        UpdateDisplayBufferForLabel('O','P');
        DelayWithDisplay(1000);

        /* 2) Show O/P with live cal for up to 3s (or longer while calibrating) */
        ShowVoltageWithCalibration(0 /* AIN0 */, 3000, &cal_offset_out);

        /* 3) Label -IP for 1s */
        UpdateDisplayBufferForLabel('I','P');
        DelayWithDisplay(1000);

        /* 4) Show I/P with live cal for up to 3s (or longer while calibrating) */
        ShowVoltageWithCalibration(1 /* AIN1 */, 3000, &cal_offset_in);
    }
}
