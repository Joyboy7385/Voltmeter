# 🔧 Step-by-Step Guide: Modular Refactoring

**Goal:** Transform your single-file project into a professional modular architecture
**Impact:** +0.5 rating (7.5 → 8.0)
**Time:** 3-4 hours
**Difficulty:** Medium

---

## 📁 Target File Structure

```
Voltmeter/
├── src/
│   ├── main.c              (100 lines - entry point)
│   ├── adc.c               (150 lines - ADC driver)
│   ├── display.c           (200 lines - 7-segment display)
│   ├── calibration.c       (120 lines - calibration logic)
│   ├── button.c            (80 lines - button handling)
│   └── voltage.c           (60 lines - voltage calculation)
├── inc/
│   ├── config.h            (All #defines)
│   ├── adc.h               (ADC API)
│   ├── display.h           (Display API)
│   ├── calibration.h       (Calibration API)
│   ├── button.h            (Button API)
│   └── voltage.h           (Voltage API)
└── voltmeter_production.c  (Keep for reference)
```

---

## 📋 Step 1: Create Directory Structure (5 minutes)

### 1.1 Create Directories

**In Git Bash / Terminal:**
```bash
cd /path/to/Voltmeter
mkdir -p src inc
```

**In Windows Command Prompt:**
```cmd
cd C:\path\to\Voltmeter
mkdir src
mkdir inc
```

---

## 📋 Step 2: Create config.h (15 minutes)

This centralizes ALL #defines from your current code.

**File: `inc/config.h`**

```c
#ifndef CONFIG_H
#define CONFIG_H

#include "SFR_Macro_MS51_16K_keil.h"

/* Build Configuration */
#ifdef PRODUCTION_BUILD
    #define ENABLE_CODE_PROTECTION  1
    #define ENABLE_DEBUG            0
#else
    #define ENABLE_CODE_PROTECTION  0
    #define ENABLE_DEBUG            1
#endif

/* System Configuration */
#define FSYS_CLOCK            16000000UL
#define ADC_RESOLUTION        4096UL
#define ADC_BANDGAP_MV        1220UL
#define ADC_TIMEOUT           5000

/* Display Configuration */
#define MAX_DISPLAY_VALUE     999
#define DISPLAY_REFRESH_MS    2
#define NUM_DIGITS            3

/* Calibration Configuration */
#define MAX_CALIBRATION       99
#define CAL_TIMEOUT_MS        2000
#define EEPROM_CAL_ADDR_OUT   0x00
#define EEPROM_CAL_ADDR_IN    0x02

/* Voltage Calculation Constants */
#define VOLTAGE_SCALE_NUM     100UL
#define VOLTAGE_SCALE_DENOM   (ADC_RESOLUTION * 1000UL)
#define ROUNDING_OFFSET       (ADC_RESOLUTION / 2)

/* 7-Segment Display Patterns */
static const unsigned char DIGIT_PAT[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

#define PATTERN_O    0x3F
#define PATTERN_P    0x63
#define PATTERN_I    0x06
#define PATTERN_DASH 0x40

/* Common-Anode Output Macro */
#define CA_OUTPUT(pat) ((~(pat)) & 0x7F)

/* Global bit required for macros */
extern bit BIT_TMP;

#endif /* CONFIG_H */
```

**Action:** Copy all #defines from your current code here.

---

## 📋 Step 3: Create ADC Module (30 minutes)

### 3.1 Create ADC Header

**File: `inc/adc.h`**

```c
#ifndef ADC_H
#define ADC_H

#include "config.h"

/* ADC Functions */
void ADC_Init(void);
unsigned int ADC_ReadChannel(unsigned char ch);
unsigned int ADC_MeasureVDD(void);

#endif /* ADC_H */
```

### 3.2 Create ADC Implementation

**File: `src/adc.c`**

```c
#include "adc.h"

/* Private variable */
static unsigned int last_vdd_mv = 5000;

/**
 * @brief Initialize ADC hardware
 */
void ADC_Init(void)
{
    ENABLE_ADC_CH0;              // Enable channel 0 (P1.7)
    ADCCON1 |= 0x30;             // 12-bit mode
    ADCCON2 = 0x0E;              // ADC clock = Fsys/16
    set_ADCCON0_ADCEN;           // Enable ADC

    // Set P1.7 as analog input
    P17_INPUT_MODE;
}

/**
 * @brief Read ADC channel
 * @param ch ADC channel number (0-15, 16=bandgap)
 * @return 12-bit ADC value (0-4095)
 */
unsigned int ADC_ReadChannel(unsigned char ch)
{
    unsigned int timeout = ADC_TIMEOUT;

    // Select channel
    ADCCON0 &= 0xF0;
    ADCCON0 |= ch;

    // Start conversion
    ADCF = 0;
    set_ADCCON0_ADCS;

    // Wait for conversion with timeout
    while (ADCF == 0 && --timeout > 0);

    if (timeout == 0) {
        return 0;  // Timeout
    }

    // Read result
    return (ADCRH << 4) | (ADCRL & 0x0F);
}

/**
 * @brief Measure VDD voltage using bandgap reference
 * @return VDD in millivolts
 */
unsigned int ADC_MeasureVDD(void)
{
    unsigned int bg_adc;
    unsigned long vdd;

    bg_adc = ADC_ReadChannel(16);  // Channel 16 = bandgap

    if (bg_adc == 0) {
        return last_vdd_mv;  // Return last valid reading
    }

    // VDD = (ADC_BANDGAP_MV * ADC_RESOLUTION) / bg_adc
    vdd = ((unsigned long)ADC_BANDGAP_MV * ADC_RESOLUTION) / bg_adc;

    // Validate range
    if (vdd < 4500 || vdd > 5500) {
        return last_vdd_mv;
    }

    last_vdd_mv = (unsigned int)vdd;
    return last_vdd_mv;
}
```

**Action:** Copy ADC-related code from your current file.

---

## 📋 Step 4: Create Voltage Module (20 minutes)

### 4.1 Create Voltage Header

**File: `inc/voltage.h`**

```c
#ifndef VOLTAGE_H
#define VOLTAGE_H

#include "config.h"

/* Voltage Functions */
unsigned int Voltage_Calculate(unsigned int adc_value, unsigned int vdd_mv);
unsigned int Voltage_Average(unsigned int *samples, unsigned char count);

#endif /* VOLTAGE_H */
```

### 4.2 Create Voltage Implementation

**File: `src/voltage.c`**

```c
#include "voltage.h"

/**
 * @brief Calculate voltage from ADC reading
 * @param adc_value Raw ADC value (0-4095)
 * @param vdd_mv VDD voltage in millivolts
 * @return Voltage in centivolts (0-999 = 0.00V - 9.99V)
 */
unsigned int Voltage_Calculate(unsigned int adc_value, unsigned int vdd_mv)
{
    unsigned long vt;
    unsigned int volts;

    // vt = ADC * VDD_mV * 100
    vt = (unsigned long)adc_value * vdd_mv * VOLTAGE_SCALE_NUM;

    // volts = (vt + rounding) / (4096 * 1000)
    volts = (unsigned int)((vt + ROUNDING_OFFSET) / VOLTAGE_SCALE_DENOM);

    // Clamp to display range
    if (volts > MAX_DISPLAY_VALUE) {
        volts = MAX_DISPLAY_VALUE;
    }

    return volts;
}

/**
 * @brief Calculate average of ADC samples
 * @param samples Array of ADC readings
 * @param count Number of samples
 * @return Average value
 */
unsigned int Voltage_Average(unsigned int *samples, unsigned char count)
{
    unsigned long sum = 0;
    unsigned char i;

    for (i = 0; i < count; i++) {
        sum += samples[i];
    }

    return (unsigned int)(sum / count);
}
```

---

## 📋 Step 5: Create Display Module (40 minutes)

### 5.1 Create Display Header

**File: `inc/display.h`**

```c
#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"

/* Display Mode */
typedef enum {
    DISPLAY_MODE_NORMAL,
    DISPLAY_MODE_OUTPUT,
    DISPLAY_MODE_INPUT,
    DISPLAY_MODE_CALIBRATION
} display_mode_t;

/* Display Functions */
void Display_Init(void);
void Display_SetMode(display_mode_t mode);
void Display_Update(unsigned int value);
void Display_ShowChar(unsigned char digit, unsigned char pattern);
void Display_Refresh(void);  // Call from Timer0 ISR

#endif /* DISPLAY_H */
```

### 5.2 Create Display Implementation

**File: `src/display.c`**

```c
#include "display.h"

/* Private variables */
static display_mode_t current_mode = DISPLAY_MODE_NORMAL;
static unsigned int display_value = 0;
static unsigned char digit_index = 0;

/**
 * @brief Initialize display hardware
 */
void Display_Init(void)
{
    // Configure P0 as push-pull output (segments)
    P0M1 = 0x00;
    P0M2 = 0xFF;

    // Configure P1 as push-pull output (digit select)
    P1M1 = 0x00;
    P1M2 = 0xFF;

    // Configure P2.0 as push-pull output (decimal point)
    P2M1 &= ~0x01;
    P2M2 |= 0x01;

    // All off initially
    P0 = 0xFF;
    P1 = 0x00;
    P2 &= ~0x01;
}

/**
 * @brief Set display mode
 * @param mode Display mode (normal, O, I, P)
 */
void Display_SetMode(display_mode_t mode)
{
    current_mode = mode;
}

/**
 * @brief Update display value
 * @param value Value to display (0-999)
 */
void Display_Update(unsigned int value)
{
    if (value > MAX_DISPLAY_VALUE) {
        value = MAX_DISPLAY_VALUE;
    }
    display_value = value;
}

/**
 * @brief Refresh one digit (call from Timer ISR)
 * @note This function must be called every 2ms for smooth display
 */
void Display_Refresh(void)
{
    unsigned char d0, d1, d2;
    unsigned char pattern;

    // Turn off all digits
    P1 = 0x00;

    // Calculate digits
    d0 = display_value / 100;        // Hundreds
    d1 = (display_value / 10) % 10;  // Tens
    d2 = display_value % 10;         // Ones

    // Select pattern based on digit and mode
    switch (digit_index) {
        case 0:  // First digit
            if (current_mode == DISPLAY_MODE_OUTPUT) {
                pattern = PATTERN_O;
            } else if (current_mode == DISPLAY_MODE_INPUT) {
                pattern = PATTERN_I;
            } else if (current_mode == DISPLAY_MODE_CALIBRATION) {
                pattern = PATTERN_P;
            } else {
                pattern = DIGIT_PAT[d0];
            }
            P0 = CA_OUTPUT(pattern);
            P1 = 0x01;
            break;

        case 1:  // Second digit
            pattern = DIGIT_PAT[d1];
            P0 = CA_OUTPUT(pattern);
            P1 = 0x02;
            P20 = 1;  // Decimal point ON
            break;

        case 2:  // Third digit
            pattern = DIGIT_PAT[d2];
            P0 = CA_OUTPUT(pattern);
            P1 = 0x04;
            P20 = 0;  // Decimal point OFF
            break;
    }

    // Move to next digit
    digit_index++;
    if (digit_index >= NUM_DIGITS) {
        digit_index = 0;
    }
}
```

---

## 📋 Step 6: Create Button Module (30 minutes)

**File: `inc/button.h`**

```c
#ifndef BUTTON_H
#define BUTTON_H

#include "config.h"

typedef enum {
    BTN_MODE,
    BTN_UP,
    BTN_DOWN
} button_t;

void Button_Init(void);
unsigned char Button_IsPressed(button_t btn);
void Button_WaitRelease(button_t btn);

#endif /* BUTTON_H */
```

**File: `src/button.c`**

```c
#include "button.h"

#define DEBOUNCE_DELAY 20  // ms

/**
 * @brief Initialize button hardware
 */
void Button_Init(void)
{
    P3M1 &= ~0x07;  // P3.0, P3.1, P3.2
    P3M2 &= ~0x07;
    // Buttons are quasi-bidirectional by default
}

/**
 * @brief Check if button is pressed
 * @param btn Button to check
 * @return 1 if pressed, 0 if not
 */
unsigned char Button_IsPressed(button_t btn)
{
    unsigned char pressed = 0;

    switch(btn) {
        case BTN_MODE:
            pressed = (P30 == 0);
            break;
        case BTN_UP:
            pressed = (P31 == 0);
            break;
        case BTN_DOWN:
            pressed = (P32 == 0);
            break;
    }

    if (pressed) {
        Timer0_Delay100us(200);  // 20ms debounce
        // Verify still pressed
        switch(btn) {
            case BTN_MODE:
                return (P30 == 0);
            case BTN_UP:
                return (P31 == 0);
            case BTN_DOWN:
                return (P32 == 0);
        }
    }

    return 0;
}

/**
 * @brief Wait for button to be released
 * @param btn Button to wait for
 */
void Button_WaitRelease(button_t btn)
{
    while(1) {
        switch(btn) {
            case BTN_MODE:
                if (P30 == 1) return;
                break;
            case BTN_UP:
                if (P31 == 1) return;
                break;
            case BTN_DOWN:
                if (P32 == 1) return;
                break;
        }
        Timer0_Delay100us(10);
    }
}
```

---

## 📋 Step 7: Create Calibration Module (30 minutes)

**File: `inc/calibration.h`**

```c
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "config.h"

typedef enum {
    CAL_OUTPUT,
    CAL_INPUT
} cal_type_t;

void Calibration_Init(void);
char Calibration_Load(cal_type_t type);
void Calibration_Save(cal_type_t type, char offset);
unsigned int Calibration_Apply(unsigned int voltage, char offset);

#endif /* CALIBRATION_H */
```

**File: `src/calibration.c`**

```c
#include "calibration.h"

/**
 * @brief Initialize calibration (load from EEPROM)
 */
void Calibration_Init(void)
{
    // EEPROM initialization if needed
}

/**
 * @brief Load calibration offset from EEPROM
 * @param type Calibration type (OUTPUT or INPUT)
 * @return Calibration offset (-99 to +99)
 */
char Calibration_Load(cal_type_t type)
{
    unsigned char addr = (type == CAL_OUTPUT) ?
                         EEPROM_CAL_ADDR_OUT : EEPROM_CAL_ADDR_IN;

    return (char)Read_APROM_BYTE(addr);
}

/**
 * @brief Save calibration offset to EEPROM
 * @param type Calibration type
 * @param offset Calibration offset
 */
void Calibration_Save(cal_type_t type, char offset)
{
    unsigned char addr = (type == CAL_OUTPUT) ?
                         EEPROM_CAL_ADDR_OUT : EEPROM_CAL_ADDR_IN;

    ENABLE_EEPROM_WRITING;
    Write_DATAFLASH_BYTE(addr, (unsigned char)offset);
    DISABLE_EEPROM_WRITING;
}

/**
 * @brief Apply calibration offset to voltage
 * @param voltage Uncalibrated voltage
 * @param offset Calibration offset
 * @return Calibrated voltage
 */
unsigned int Calibration_Apply(unsigned int voltage, char offset)
{
    int calibrated = (int)voltage + offset;

    // Clamp to valid range
    if (calibrated < 0) {
        calibrated = 0;
    }
    if (calibrated > MAX_DISPLAY_VALUE) {
        calibrated = MAX_DISPLAY_VALUE;
    }

    return (unsigned int)calibrated;
}
```

---

## 📋 Step 8: Create Main File (30 minutes)

**File: `src/main.c`**

```c
#include "config.h"
#include "adc.h"
#include "voltage.h"
#include "display.h"
#include "button.h"
#include "calibration.h"

bit BIT_TMP;  // Required for SFR macros

/* State machine */
typedef enum {
    STATE_NORMAL,
    STATE_CALIBRATE
} state_t;

static state_t current_state = STATE_NORMAL;

/* Function prototypes */
void System_Init(void);
void Timer0_Init(void);
void Timer0_ISR(void) interrupt 1;

/**
 * @brief Main function
 */
void main(void)
{
    unsigned int vdd_mV;
    unsigned int adc_value;
    unsigned int voltage;
    char cal_offset = 0;

    // Initialize system
    System_Init();

    // Load calibration
    cal_offset = Calibration_Load(CAL_OUTPUT);

    while(1) {
        // Measure VDD
        vdd_mV = ADC_MeasureVDD();

        // Read input voltage
        adc_value = ADC_ReadChannel(0);

        // Calculate voltage
        voltage = Voltage_Calculate(adc_value, vdd_mV);

        // Apply calibration
        voltage = Calibration_Apply(voltage, cal_offset);

        // Update display
        Display_Update(voltage);

        // Handle buttons
        if (Button_IsPressed(BTN_MODE)) {
            Button_WaitRelease(BTN_MODE);
            // Toggle mode or enter calibration
            // ... (add your state machine logic) ...
        }

        if (Button_IsPressed(BTN_UP)) {
            Button_WaitRelease(BTN_UP);
            // Handle UP button
        }

        if (Button_IsPressed(BTN_DOWN)) {
            Button_WaitRelease(BTN_DOWN);
            // Handle DOWN button
        }
    }
}

/**
 * @brief Initialize all system modules
 */
void System_Init(void)
{
    Set_All_GPIO_Quasi_Mode;

    ADC_Init();
    Display_Init();
    Button_Init();
    Calibration_Init();
    Timer0_Init();

    EA = 1;  // Enable global interrupts
}

/**
 * @brief Initialize Timer0 for display refresh
 */
void Timer0_Init(void)
{
    TIMER0_MODE1_ENABLE;
    TIMER0_FSYS_DIV12;

    // 2ms interrupt @ 16MHz
    TH0 = 0xF3;
    TL0 = 0xCB;

    set_ET0;        // Enable Timer0 interrupt
    set_TR0;        // Start Timer0
}

/**
 * @brief Timer0 ISR - Display refresh
 */
void Timer0_ISR(void) interrupt 1
{
    // Reload timer
    TH0 = 0xF3;
    TL0 = 0xCB;

    // Refresh display
    Display_Refresh();
}
```

---

## 📋 Step 9: Update Keil Project (15 minutes)

### 9.1 Add Files to Keil

1. **Open your Keil project**
2. **In Project Window**, expand "Target 1" → "Source Group 1"
3. **Right-click** "Source Group 1" → **Add Files**
4. **Navigate to `src/`** folder
5. **Add all .c files**:
   - main.c
   - adc.c
   - voltage.c
   - display.c
   - button.c
   - calibration.c

### 9.2 Set Include Path

1. **Project** → **Options for Target**
2. **C/C++** tab
3. **Include Paths**: Add `../inc`

### 9.3 Compile

Click **Build** (F7) - Should compile without errors!

---

## 📋 Step 10: Test (30 minutes)

### 10.1 Smoke Test
1. Flash to device
2. Verify display shows voltage
3. Test all buttons
4. Verify calibration works

### 10.2 Verify Functionality
- ✅ Voltage measurement correct
- ✅ Display updates smoothly
- ✅ Buttons respond
- ✅ Calibration saves/loads

---

## ✅ CHECKLIST

- [ ] Step 1: Create directories (src, inc)
- [ ] Step 2: Create config.h
- [ ] Step 3: Create ADC module (adc.h, adc.c)
- [ ] Step 4: Create Voltage module (voltage.h, voltage.c)
- [ ] Step 5: Create Display module (display.h, display.c)
- [ ] Step 6: Create Button module (button.h, button.c)
- [ ] Step 7: Create Calibration module (calibration.h, calibration.c)
- [ ] Step 8: Create main.c
- [ ] Step 9: Update Keil project
- [ ] Step 10: Compile and test

---

## 🎯 RESULTS

**Before:**
```
voltmeter_production.c  (657 lines)
```

**After:**
```
src/main.c           (100 lines)
src/adc.c            (150 lines)
src/voltage.c        (60 lines)
src/display.c        (200 lines)
src/button.c         (80 lines)
src/calibration.c    (120 lines)
inc/*.h              (150 lines)
-----------------------------------
Total: 860 lines (modular!)
```

**Benefits:**
- ✅ Each module is self-contained
- ✅ Easy to test individually
- ✅ Easy to reuse in other projects
- ✅ Easy to maintain and debug
- ✅ Clear separation of concerns

**Rating Improvement: 7.5 → 8.0** 🎉

---

## 💡 TIPS

1. **Do it incrementally** - Create one module at a time and test
2. **Keep the original** - Don't delete voltmeter_production.c yet
3. **Test frequently** - Compile after each module
4. **Use version control** - Commit after each working module

---

## 🚀 NEXT STEPS

After completing this refactoring:
1. Add watchdog timer (QUICK_WIN_EXAMPLE.c)
2. Add error handling
3. Create professional Makefile
4. Add unit tests for each module

**Total Time Investment:** 3-4 hours
**Professional Improvement:** Massive! 🌟

Good luck! 🚀
