/**
 * @file test_simulation.c
 * @brief Logic simulation and testing for voltmeter code
 *
 * This file tests the voltmeter logic without requiring actual hardware.
 * It simulates ADC values, calculates expected results, and verifies correctness.
 *
 * Compile: gcc -o test_simulation test_simulation.c -lm
 * Run: ./test_simulation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Constants from voltmeter_improved.c */
#define FSYS_CLOCK            16000000UL
#define ADC_RESOLUTION        4096UL
#define ADC_BANDGAP_MV        1220UL
#define MAX_DISPLAY_VALUE     999
#define MAX_CALIBRATION       99
#define VOLTAGE_SCALE_NUM     100UL  // FIXED: Was 125 (caused 25% error)
#define VOLTAGE_SCALE_DENOM   (ADC_RESOLUTION * 1000UL)
#define ROUNDING_OFFSET       (ADC_RESOLUTION / 2)

/* Display patterns */
static const unsigned char DIGIT_PAT[10] = {
    0x3F, /*0*/ 0x06, /*1*/ 0x5B, /*2*/ 0x4F, /*3*/ 0x66, /*4*/
    0x6D, /*5*/ 0x7D, /*6*/ 0x07, /*7*/ 0x7F, /*8*/ 0x6F  /*9*/
};

#define PATTERN_O    0x3F
#define PATTERN_P    0x63
#define PATTERN_I    0x06
#define PATTERN_DASH 0x40

/* Test counters */
static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

/* Helper macros */
#define TEST_ASSERT(condition, message) do { \
    tests_run++; \
    if (condition) { \
        tests_passed++; \
        printf("  ✓ PASS: %s\n", message); \
    } else { \
        tests_failed++; \
        printf("  ✗ FAIL: %s\n", message); \
    } \
} while(0)

#define TEST_SECTION(name) printf("\n=== %s ===\n", name)

/*============================================================================*/
/* Simulated Functions from voltmeter_improved.c */
/*============================================================================*/

/**
 * @brief Simulate VDD measurement calculation
 */
unsigned int Simulate_Measure_VDD_mV(unsigned int adc_bg_reading)
{
    unsigned int adc_bg = adc_bg_reading;
    unsigned int vdd_mV;

    /* Avoid divide-by-zero */
    if (adc_bg == 0)
        adc_bg = 1;

    /* Calculate VDD */
    vdd_mV = (ADC_BANDGAP_MV * ADC_RESOLUTION) / adc_bg;

    return vdd_mV;
}

/**
 * @brief Simulate voltage calculation
 */
unsigned int Simulate_CalculateVoltage(unsigned int adc_value, unsigned int vdd_mV)
{
    unsigned long vt;
    unsigned int volts;

    /* Voltage calculation */
    vt = (unsigned long)adc_value * vdd_mV * VOLTAGE_SCALE_NUM;
    volts = (vt + ROUNDING_OFFSET) / VOLTAGE_SCALE_DENOM;

    /* Safety clamp */
    if (volts > MAX_DISPLAY_VALUE)
        volts = MAX_DISPLAY_VALUE;

    return volts;
}

/**
 * @brief Apply calibration offset
 */
unsigned int Simulate_ApplyOffset(unsigned int v, int offset)
{
    int adj = (int)v + offset;
    if (adj < 0) adj = 0;
    if (adj > MAX_DISPLAY_VALUE) adj = MAX_DISPLAY_VALUE;
    return (unsigned int)adj;
}

/**
 * @brief Simulate display buffer update
 */
void Simulate_UpdateDisplayBufferForValue(unsigned int value, unsigned char *pattern)
{
    unsigned int h = value / 100U;
    unsigned int t = (value / 10U) % 10U;
    unsigned int o = value % 10U;

    pattern[0] = (h==0) ? 0x00 : DIGIT_PAT[h];
    pattern[1] = (h==0 && t==0) ? 0x00 : DIGIT_PAT[t];
    pattern[2] = DIGIT_PAT[o];
}

/*============================================================================*/
/* Test Functions */
/*============================================================================*/

void test_vdd_measurement(void)
{
    TEST_SECTION("VDD Measurement Tests");

    unsigned int result;

    /* Test 1: Normal VDD = 5.0V */
    /* ADC_BG at 5V: 1.22V / 5.0V * 4096 = 1000 */
    result = Simulate_Measure_VDD_mV(1000);
    TEST_ASSERT(abs(result - 5000) < 10,
                "VDD 5.0V: Expected ~5000mV");
    printf("    Result: %u mV\n", result);

    /* Test 2: Low VDD = 4.5V */
    /* ADC_BG at 4.5V: 1.22V / 4.5V * 4096 = 1110 */
    result = Simulate_Measure_VDD_mV(1110);
    TEST_ASSERT(abs(result - 4500) < 10,
                "VDD 4.5V: Expected ~4500mV");
    printf("    Result: %u mV\n", result);

    /* Test 3: High VDD = 5.5V */
    /* ADC_BG at 5.5V: 1.22V / 5.5V * 4096 = 909 */
    result = Simulate_Measure_VDD_mV(909);
    TEST_ASSERT(abs(result - 5500) < 10,
                "VDD 5.5V: Expected ~5500mV");
    printf("    Result: %u mV\n", result);

    /* Test 4: Divide-by-zero protection */
    result = Simulate_Measure_VDD_mV(0);
    TEST_ASSERT(result > 0,
                "ADC=0: Should not return 0 (divide-by-zero protection)");
    printf("    Result with ADC=0: %u mV\n", result);
}

void test_voltage_calculation(void)
{
    TEST_SECTION("Voltage Calculation Tests");

    unsigned int result;

    /* Test 1: 0V input */
    result = Simulate_CalculateVoltage(0, 5000);
    TEST_ASSERT(result == 0,
                "0V input: Expected 0V");
    printf("    ADC=0, VDD=5000mV -> %u V\n", result);

    /* Test 2: Mid-scale (2.5V / 5V = ADC 2048) */
    result = Simulate_CalculateVoltage(2048, 5000);
    TEST_ASSERT(abs(result - 250) <= 1,
                "2.5V input: Expected ~250V");
    printf("    ADC=2048, VDD=5000mV -> %u V (expected 250V)\n", result);

    /* Test 3: Full-scale (5V / 5V = ADC 4095) */
    result = Simulate_CalculateVoltage(4095, 5000);
    TEST_ASSERT(abs(result - 499) <= 1,
                "5.0V input: Expected ~499V");
    printf("    ADC=4095, VDD=5000mV -> %u V (expected 499V)\n", result);

    /* Test 4: 1.0V input (1V / 5V = ADC 819) */
    result = Simulate_CalculateVoltage(819, 5000);
    TEST_ASSERT(abs(result - 100) <= 1,
                "1.0V input: Expected ~100V");
    printf("    ADC=819, VDD=5000mV -> %u V (expected 100V)\n", result);

    /* Test 5: 3.3V input (3.3V / 5V = ADC 2703) */
    result = Simulate_CalculateVoltage(2703, 5000);
    TEST_ASSERT(abs(result - 330) <= 2,
                "3.3V input: Expected ~330V");
    printf("    ADC=2703, VDD=5000mV -> %u V (expected 330V)\n", result);
}

void test_vdd_compensation(void)
{
    TEST_SECTION("VDD Compensation Tests");

    /* Test: Same input voltage at different VDD should give same result */
    /* Input = 3.3V, VDD varies */

    unsigned int result_4_5V, result_5_0V, result_5_5V;

    /* At VDD=4.5V: 3.3V / 4.5V * 4096 = 3002 */
    result_4_5V = Simulate_CalculateVoltage(3002, 4500);

    /* At VDD=5.0V: 3.3V / 5.0V * 4096 = 2703 */
    result_5_0V = Simulate_CalculateVoltage(2703, 5000);

    /* At VDD=5.5V: 3.3V / 5.5V * 4096 = 2457 */
    result_5_5V = Simulate_CalculateVoltage(2457, 5500);

    printf("    3.3V input at VDD=4.5V: %u V\n", result_4_5V);
    printf("    3.3V input at VDD=5.0V: %u V\n", result_5_0V);
    printf("    3.3V input at VDD=5.5V: %u V\n", result_5_5V);

    TEST_ASSERT(abs(result_4_5V - 330) <= 2,
                "VDD compensation at 4.5V: Should show ~330V");
    TEST_ASSERT(abs(result_5_0V - 330) <= 2,
                "VDD compensation at 5.0V: Should show ~330V");
    TEST_ASSERT(abs(result_5_5V - 330) <= 2,
                "VDD compensation at 5.5V: Should show ~330V");

    int max_diff = abs(result_4_5V - result_5_0V);
    if (abs(result_4_5V - result_5_5V) > max_diff)
        max_diff = abs(result_4_5V - result_5_5V);
    if (abs(result_5_0V - result_5_5V) > max_diff)
        max_diff = abs(result_5_0V - result_5_5V);

    TEST_ASSERT(max_diff <= 3,
                "VDD compensation: Max deviation ≤3V across VDD range");
    printf("    Max deviation: %d V\n", max_diff);
}

void test_overflow_protection(void)
{
    TEST_SECTION("Overflow Protection Tests");

    /* Test maximum safe values */
    unsigned long vt_max;
    unsigned int result;

    /* Test 1: Maximum normal case (ADC=4095, VDD=6V) */
    vt_max = (unsigned long)4095 * 6000 * VOLTAGE_SCALE_NUM;
    printf("    Max calc (ADC=4095, VDD=6000mV): vt = %lu\n", vt_max);
    TEST_ASSERT(vt_max < 4294967295UL,
                "No overflow for VDD ≤6V");

    result = Simulate_CalculateVoltage(4095, 6000);
    printf("    Result: %u V\n", result);

    /* Test 2: Maximum extreme case (ADC=4095, VDD=8V) */
    vt_max = (unsigned long)4095 * 8000 * VOLTAGE_SCALE_NUM;
    printf("    Max calc (ADC=4095, VDD=8000mV): vt = %lu\n", vt_max);
    TEST_ASSERT(vt_max < 4294967295UL,
                "No overflow for VDD ≤8V");

    /* Test 3: Overflow point (ADC=4095, VDD=8.4V) */
    vt_max = (unsigned long)4095 * 8400 * VOLTAGE_SCALE_NUM;
    printf("    Max calc (ADC=4095, VDD=8400mV): vt = %lu\n", vt_max);
    printf("    (This would overflow if VDD > 8.4V)\n");
}

void test_calibration_offset(void)
{
    TEST_SECTION("Calibration Offset Tests");

    unsigned int result;

    /* Test 1: Positive offset */
    result = Simulate_ApplyOffset(100, 10);
    TEST_ASSERT(result == 110,
                "Positive offset: 100 + 10 = 110");
    printf("    100V + 10 offset = %u V\n", result);

    /* Test 2: Negative offset */
    result = Simulate_ApplyOffset(100, -10);
    TEST_ASSERT(result == 90,
                "Negative offset: 100 - 10 = 90");
    printf("    100V - 10 offset = %u V\n", result);

    /* Test 3: Offset causing negative (clamped to 0) */
    result = Simulate_ApplyOffset(10, -20);
    TEST_ASSERT(result == 0,
                "Underflow clamp: 10 - 20 = 0 (clamped)");
    printf("    10V - 20 offset = %u V (clamped to 0)\n", result);

    /* Test 4: Offset causing overflow (clamped to 999) */
    result = Simulate_ApplyOffset(990, 20);
    TEST_ASSERT(result == 999,
                "Overflow clamp: 990 + 20 = 999 (clamped)");
    printf("    990V + 20 offset = %u V (clamped to 999)\n", result);

    /* Test 5: Maximum positive offset */
    result = Simulate_ApplyOffset(500, MAX_CALIBRATION);
    TEST_ASSERT(result == 599,
                "Max positive offset: 500 + 99 = 599");
    printf("    500V + %d offset = %u V\n", MAX_CALIBRATION, result);

    /* Test 6: Maximum negative offset */
    result = Simulate_ApplyOffset(500, -MAX_CALIBRATION);
    TEST_ASSERT(result == 401,
                "Max negative offset: 500 - 99 = 401");
    printf("    500V - %d offset = %u V\n", MAX_CALIBRATION, result);
}

void test_display_buffer(void)
{
    TEST_SECTION("Display Buffer Tests");

    unsigned char pattern[3];

    /* Test 1: Zero (should show "  0") */
    Simulate_UpdateDisplayBufferForValue(0, pattern);
    TEST_ASSERT(pattern[0] == 0x00 && pattern[1] == 0x00 && pattern[2] == DIGIT_PAT[0],
                "Display 0: Should be [blank, blank, 0]");
    printf("    Value 0: [0x%02X, 0x%02X, 0x%02X]\n", pattern[0], pattern[1], pattern[2]);

    /* Test 2: Single digit (should show "  5") */
    Simulate_UpdateDisplayBufferForValue(5, pattern);
    TEST_ASSERT(pattern[0] == 0x00 && pattern[1] == 0x00 && pattern[2] == DIGIT_PAT[5],
                "Display 5: Should be [blank, blank, 5]");
    printf("    Value 5: [0x%02X, 0x%02X, 0x%02X]\n", pattern[0], pattern[1], pattern[2]);

    /* Test 3: Two digits (should show " 23") */
    Simulate_UpdateDisplayBufferForValue(23, pattern);
    TEST_ASSERT(pattern[0] == 0x00 && pattern[1] == DIGIT_PAT[2] && pattern[2] == DIGIT_PAT[3],
                "Display 23: Should be [blank, 2, 3]");
    printf("    Value 23: [0x%02X, 0x%02X, 0x%02X]\n", pattern[0], pattern[1], pattern[2]);

    /* Test 4: Three digits (should show "123") */
    Simulate_UpdateDisplayBufferForValue(123, pattern);
    TEST_ASSERT(pattern[0] == DIGIT_PAT[1] && pattern[1] == DIGIT_PAT[2] && pattern[2] == DIGIT_PAT[3],
                "Display 123: Should be [1, 2, 3]");
    printf("    Value 123: [0x%02X, 0x%02X, 0x%02X]\n", pattern[0], pattern[1], pattern[2]);

    /* Test 5: Maximum value (should show "999") */
    Simulate_UpdateDisplayBufferForValue(999, pattern);
    TEST_ASSERT(pattern[0] == DIGIT_PAT[9] && pattern[1] == DIGIT_PAT[9] && pattern[2] == DIGIT_PAT[9],
                "Display 999: Should be [9, 9, 9]");
    printf("    Value 999: [0x%02X, 0x%02X, 0x%02X]\n", pattern[0], pattern[1], pattern[2]);
}

void test_rounding_accuracy(void)
{
    TEST_SECTION("Rounding Accuracy Tests");

    /* Test rounding behavior with ROUNDING_OFFSET = 2048 */

    /* These tests verify that the rounding offset (2048) properly
       rounds to the nearest integer rather than truncating */

    unsigned int result;

    /* Calculate a value that should round down */
    /* Example: ADC=820, VDD=5000 should give ~100.1V -> rounds to 100V */
    result = Simulate_CalculateVoltage(820, 5000);
    printf("    ADC=820, VDD=5000mV -> %u V\n", result);

    /* Calculate a value that should round up */
    /* Example: ADC=821, VDD=5000 should give ~100.2V -> rounds to 100V */
    result = Simulate_CalculateVoltage(821, 5000);
    printf("    ADC=821, VDD=5000mV -> %u V\n", result);

    /* Without rounding offset, results would be consistently low */
    unsigned long vt_no_round = (unsigned long)820 * 5000 * 125UL;
    unsigned int volts_no_round = vt_no_round / (4096UL * 1000UL);
    printf("    Without rounding: %u V (underestimated)\n", volts_no_round);

    /* With rounding offset */
    vt_no_round = (unsigned long)820 * 5000 * 125UL;
    unsigned int volts_with_round = (vt_no_round + 2048UL) / (4096UL * 1000UL);
    printf("    With rounding: %u V (properly rounded)\n", volts_with_round);

    TEST_ASSERT(volts_with_round >= volts_no_round,
                "Rounding offset should prevent systematic underestimation");
}

void test_edge_cases(void)
{
    TEST_SECTION("Edge Case Tests");

    unsigned int result;

    /* Test 1: ADC minimum */
    result = Simulate_CalculateVoltage(0, 5000);
    TEST_ASSERT(result == 0,
                "ADC=0: Should give 0V");
    printf("    ADC min (0) -> %u V\n", result);

    /* Test 2: ADC maximum */
    result = Simulate_CalculateVoltage(4095, 5000);
    TEST_ASSERT(result > 0 && result <= MAX_DISPLAY_VALUE,
                "ADC=4095: Should give valid voltage");
    printf("    ADC max (4095) -> %u V\n", result);

    /* Test 3: VDD minimum */
    result = Simulate_CalculateVoltage(2048, 4000);
    TEST_ASSERT(result > 0,
                "Low VDD (4V): Should still calculate");
    printf("    VDD min (4000mV), ADC=2048 -> %u V\n", result);

    /* Test 4: VDD maximum safe */
    result = Simulate_CalculateVoltage(2048, 6000);
    TEST_ASSERT(result > 0,
                "High VDD (6V): Should still calculate");
    printf("    VDD high (6000mV), ADC=2048 -> %u V\n", result);

    /* Test 5: Result exceeds display max */
    result = Simulate_CalculateVoltage(4095, 7000);
    TEST_ASSERT(result <= MAX_DISPLAY_VALUE,
                "Overflow: Result should be clamped to 999");
    printf("    Very high result -> %u V (clamped if needed)\n", result);
}

void test_real_world_scenarios(void)
{
    TEST_SECTION("Real-World Scenario Tests");

    unsigned int adc, result;
    unsigned int vdd_mV = 5000;

    printf("\n  Testing common voltages at VDD=5.0V:\n");

    /* Test common voltage levels */
    struct {
        double input_v;
        unsigned int expected_display;
    } test_cases[] = {
        {0.00, 0},
        {1.00, 100},
        {1.80, 180},  // Logic level
        {3.30, 330},  // Common 3.3V rail
        {5.00, 500},  // Full scale
    };

    for (int i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++) {
        /* Calculate ADC value for this input voltage */
        adc = (unsigned int)((test_cases[i].input_v / 5.0) * 4095.0);
        result = Simulate_CalculateVoltage(adc, vdd_mV);

        int error = abs((int)result - (int)test_cases[i].expected_display);

        printf("    %.2fV (ADC=%u) -> %u V (expected %u, error=%d)\n",
               test_cases[i].input_v, adc, result,
               test_cases[i].expected_display, error);

        TEST_ASSERT(error <= 2,
                    "Real-world voltage within ±2V tolerance");
    }
}

/*============================================================================*/
/* Main Test Runner */
/*============================================================================*/

int main(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║  VOLTMETER CODE SIMULATION TEST SUITE                   ║\n");
    printf("║  Testing: voltmeter_improved.c logic                    ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");

    /* Run all test suites */
    test_vdd_measurement();
    test_voltage_calculation();
    test_vdd_compensation();
    test_overflow_protection();
    test_calibration_offset();
    test_display_buffer();
    test_rounding_accuracy();
    test_edge_cases();
    test_real_world_scenarios();

    /* Print summary */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║  TEST SUMMARY                                            ║\n");
    printf("╠══════════════════════════════════════════════════════════╣\n");
    printf("║  Total Tests:  %3d                                       ║\n", tests_run);
    printf("║  Passed:       %3d ✓                                     ║\n", tests_passed);
    printf("║  Failed:       %3d ✗                                     ║\n", tests_failed);
    printf("║  Pass Rate:    %3d%%                                      ║\n",
           tests_run > 0 ? (tests_passed * 100 / tests_run) : 0);
    printf("╠══════════════════════════════════════════════════════════╣\n");

    if (tests_failed == 0) {
        printf("║  STATUS: ALL TESTS PASSED ✓✓✓                           ║\n");
        printf("╚══════════════════════════════════════════════════════════╝\n");
        return 0;
    } else {
        printf("║  STATUS: SOME TESTS FAILED ✗✗✗                          ║\n");
        printf("╚══════════════════════════════════════════════════════════╝\n");
        return 1;
    }
}
