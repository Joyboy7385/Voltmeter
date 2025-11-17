# Voltmeter Code Test Plan

**Version:** 1.0
**Date:** 2025-11-17
**Target:** voltmeter_improved.c
**MCU:** MS51FB9AE

---

## Test Overview

This document outlines comprehensive test scenarios for the voltmeter project, covering:
- ADC functionality and edge cases
- Voltage calculation accuracy
- Display output verification
- Button debouncing
- Calibration logic
- Error handling

---

## Test Environment

### Hardware Required:
- MS51FB9AE development board
- 3-digit 7-segment common-anode display
- 2x push buttons (pull-down to GND)
- Voltage sources (0-5V adjustable)
- Digital multimeter (reference)
- Oscilloscope (timing verification)
- Power supply (4.5V - 5.5V variable)

### Software Required:
- Keil C51 compiler
- Flash programmer
- Serial debug interface (optional)

---

## Test Scenarios

## Category 1: ADC Reading Tests

### Test 1.1: Basic ADC Reading
**Objective:** Verify ADC reads correct values

**Setup:**
- Apply known voltage to AIN0 (P1.7)
- VDD = 5.00V

**Test Cases:**

| Input (V) | Expected ADC | Tolerance | Status |
|-----------|--------------|-----------|--------|
| 0.00 | 0 | ±5 | |
| 1.00 | 819 | ±10 | |
| 2.50 | 2048 | ±10 | |
| 3.30 | 2703 | ±10 | |
| 5.00 | 4095 | ±5 | |

**Pass Criteria:** All readings within tolerance

---

### Test 1.2: ADC Timeout Protection
**Objective:** Verify timeout prevents infinite loops

**Setup:**
- Disconnect AVDD to simulate ADC failure
- Monitor execution time

**Expected Result:**
- Function returns 0 within 100ms
- System continues running
- No infinite loop

**Pass Criteria:** Graceful degradation, no hang

---

### Test 1.3: ADC Oversampling
**Objective:** Verify 8x oversampling reduces noise

**Setup:**
- Apply 2.50V with intentional noise (50mV p-p)
- Compare with and without averaging

**Expected Results:**
- Standard deviation reduced by ~2.8x (sqrt(8))
- Average reading stable within ±5 LSB

**Pass Criteria:** Noise reduction >60%

---

### Test 1.4: Bandgap VDD Measurement
**Objective:** Verify VDD measurement accuracy

**Setup:**
- Vary VDD from 4.5V to 5.5V
- Compare measured vs actual (multimeter)

| VDD Actual (V) | Expected Reading (mV) | Tolerance (mV) | Status |
|----------------|----------------------|----------------|--------|
| 4.50 | 4500 | ±50 | |
| 4.75 | 4750 | ±50 | |
| 5.00 | 5000 | ±50 | |
| 5.25 | 5250 | ±50 | |
| 5.50 | 5500 | ±50 | |

**Pass Criteria:** All within ±1% of actual VDD

---

## Category 2: Voltage Calculation Tests

### Test 2.1: Voltage Calculation Accuracy
**Objective:** Verify voltage conversion formula

**Setup:**
- Use known ADC values and VDD
- Calculate expected output

**Test Cases:**

| ADC Value | VDD (mV) | Expected Volts | Calculation |
|-----------|----------|----------------|-------------|
| 0 | 5000 | 0 | 0*5000*125/(4096*1000) |
| 819 | 5000 | 100 | 819*5000*125/(4096*1000) ≈ 100V |
| 2048 | 5000 | 250 | 2048*5000*125/(4096*1000) ≈ 250V |
| 4095 | 5000 | 499 | 4095*5000*125/(4096*1000) ≈ 499V |
| 4095 | 4500 | 449 | 4095*4500*125/(4096*1000) ≈ 449V |

**Verification Method:**
```c
// Manual calculation for test
vt = (unsigned long)adc * vdd_mV * 125UL;
volts = (vt + 2048UL) / (4096UL * 1000UL);
```

**Pass Criteria:** Results match calculated values ±1V

---

### Test 2.2: Rounding Accuracy
**Objective:** Verify proper rounding with 2048 offset

**Test Cases:**

| Calculation | Without Rounding | With Rounding | Correct |
|-------------|------------------|---------------|---------|
| 1023.4 mV | 1023 | 1023 | 1023 |
| 1023.5 mV | 1023 | 1024 | 1024 |
| 1023.6 mV | 1023 | 1024 | 1024 |

**Pass Criteria:** Rounding to nearest integer

---

### Test 2.3: Overflow Protection
**Objective:** Verify no integer overflow

**Test Cases:**

| ADC | VDD (mV) | Intermediate Calc | Max Value | Safe? |
|-----|----------|-------------------|-----------|-------|
| 4095 | 6000 | 3,071,250,000 | 4,294,967,295 | ✓ |
| 4095 | 7000 | 3,583,125,000 | 4,294,967,295 | ✓ |
| 4095 | 8000 | 4,096,000,000 | 4,294,967,295 | ✓ |
| 4095 | 8400 | 4,300,800,000 | 4,294,967,295 | ✗ |

**Pass Criteria:** No overflow for VDD up to 8.0V

---

### Test 2.4: VDD Compensation
**Objective:** Verify VDD compensation works correctly

**Setup:**
- Apply 3.30V to AIN0
- Vary VDD

| VDD (V) | Input (V) | Expected Display | Notes |
|---------|-----------|------------------|-------|
| 4.50 | 3.30 | 330 | Low VDD |
| 5.00 | 3.30 | 330 | Nominal |
| 5.50 | 3.30 | 330 | High VDD |

**Pass Criteria:** Display shows 330V ±2V for all VDD values

---

## Category 3: Display Tests

### Test 3.1: Numeric Display
**Objective:** Verify correct digit patterns

**Test Cases:**

| Value | Digit 1 | Digit 2 | Digit 3 | Leading Zero Suppression |
|-------|---------|---------|---------|--------------------------|
| 0 | blank | blank | 0 | ✓ |
| 5 | blank | blank | 5 | ✓ |
| 23 | blank | 2 | 3 | ✓ |
| 123 | 1 | 2 | 3 | N/A |
| 999 | 9 | 9 | 9 | N/A |

**Verification:**
```c
segPattern[0] should be 0x00 for values < 100
segPattern[1] should be 0x00 for values < 10
```

**Pass Criteria:** Correct patterns, proper zero suppression

---

### Test 3.2: Label Display
**Objective:** Verify label patterns

| Label | Call | Expected Pattern |
|-------|------|------------------|
| -OP | UpdateDisplayBufferForLabel('O','P') | [DASH, O, P] |
| -IP | UpdateDisplayBufferForLabel('I','P') | [DASH, I, P] |

**Expected Patterns:**
- DASH = 0x40
- O = 0x3F (same as '0')
- P = 0x63
- I = 0x06 (same as '1')

**Pass Criteria:** Correct segment patterns displayed

---

### Test 3.3: Common-Anode Inversion
**Objective:** Verify CA_OUTPUT macro works correctly

**Test Cases:**

| Pattern (CC) | Inverted (CA) | Bit Pattern |
|--------------|---------------|-------------|
| 0x3F (0) | 0x40 | 0100 0000 |
| 0x06 (1) | 0x79 | 0111 1001 |
| 0x5B (2) | 0x24 | 0010 0100 |

**Formula:** `CA = (~CC) & 0x7F`

**Pass Criteria:** Correct inversion, bit 7 always 0

---

### Test 3.4: Display Refresh Timing
**Objective:** Verify 1ms per digit multiplexing

**Setup:**
- Monitor DIGIT1, DIGIT2, DIGIT3 with oscilloscope
- Measure timing

**Expected:**
- Each digit ON for ~1ms
- 3ms full cycle
- ~333 Hz refresh rate

**Pass Criteria:**
- Individual digit time: 1ms ±10%
- No visible flicker
- No ghosting between digits

---

## Category 4: Button and Calibration Tests

### Test 4.1: Button Debouncing
**Objective:** Verify 20ms debounce works correctly

**Setup:**
- Generate bouncy button presses
- Monitor press detection

**Test Cases:**

| Bounce Duration | Press Detected? | Notes |
|-----------------|-----------------|-------|
| 5ms | No | Too short |
| 15ms | No | Below threshold |
| 20ms | Yes | At threshold |
| 50ms | Yes | Above threshold |

**Pass Criteria:** Only stable presses >20ms detected

---

### Test 4.2: Calibration Offset
**Objective:** Verify calibration adjustment

**Setup:**
- Start with cal_offset = 0
- Press INC/DEC buttons

**Test Cases:**

| Action | Expected Offset | Display Change |
|--------|-----------------|----------------|
| None | 0 | No change |
| Press INC 5x | +5 | +5V |
| Press DEC 3x | +2 | +2V |
| Press DEC 5x | -3 | -3V |

**Pass Criteria:** Each press adjusts offset by ±1V

---

### Test 4.3: Calibration Limits
**Objective:** Verify +/-99V limits enforced

**Test Cases:**

| Starting Offset | Button Presses | Expected Final | Actual |
|-----------------|----------------|----------------|--------|
| +98 | INC x 5 | +99 | |
| -98 | DEC x 5 | -99 | |
| +99 | INC x 10 | +99 | |
| -99 | DEC x 10 | -99 | |

**Pass Criteria:** Offset clamped at ±99

---

### Test 4.4: Calibration Mode Timeout
**Objective:** Verify 2s timeout exits cal mode

**Setup:**
- Enter cal mode (press INC/DEC)
- Wait without button presses

**Expected:**
- After 2000ms ±100ms, exit cal mode
- Return to normal display cycle

**Pass Criteria:** Timeout occurs at 2s ±5%

---

### Test 4.5: Calibration Persistence
**Objective:** Verify offset applies correctly

**Setup:**
- Set cal_offset_out = +10
- Apply 100V input

**Expected:**
- Display shows 110V (100 + 10)

**Test Cases:**

| Input (V) | Offset | Expected Display |
|-----------|--------|------------------|
| 100 | +10 | 110 |
| 100 | -10 | 90 |
| 0 | +50 | 50 |
| 500 | +50 | 550 |

**Pass Criteria:** Offset applied correctly

---

### Test 4.6: Calibration Clamping
**Objective:** Verify offset clamping prevents display overflow

**Test Cases:**

| Input | Offset | Raw Result | Expected Display | Clamped? |
|-------|--------|------------|------------------|----------|
| 990 | +20 | 1010 | 999 | Yes |
| 10 | -20 | -10 | 0 | Yes |
| 500 | +99 | 599 | 599 | No |

**Pass Criteria:** Display always in 0-999 range

---

## Category 5: Edge Cases and Error Handling

### Test 5.1: Zero ADC Reading
**Objective:** Handle ADC = 0 correctly

**Test:**
- Short AIN0 to GND
- Expected: Display shows "  0"

**Pass Criteria:** No divide-by-zero, correct display

---

### Test 5.2: Maximum ADC Reading
**Objective:** Handle ADC = 4095 correctly

**Test:**
- Connect AIN0 to VDD
- Expected: Display shows ~499V (at VDD=5V)

**Pass Criteria:** No overflow, correct calculation

---

### Test 5.3: VDD Bandgap Read = 0
**Objective:** Verify divide-by-zero protection

**Code:**
```c
if (adc_bg == 0)
    adc_bg = 1;
```

**Expected:** Function returns safe value, no crash

**Pass Criteria:** No system crash

---

### Test 5.4: Rapid Button Presses
**Objective:** Verify debounce handles rapid input

**Test:**
- Press INC button rapidly (10x in 100ms)
- Expected: Only 1-2 presses registered

**Pass Criteria:** Debounce prevents multiple triggers

---

### Test 5.5: Both Buttons Pressed Simultaneously
**Objective:** Verify both buttons can be handled

**Test:**
- Press INC and DEC at same time
- Expected: Both events registered independently

**Pass Criteria:** No conflict, both processed

---

### Test 5.6: ADC Channel Switching
**Objective:** Verify clean switch between AIN0/AIN1

**Test:**
- Apply different voltages to AIN0 and AIN1
- Monitor display during channel switch

**Expected:**
- No crosstalk between channels
- Clean switching

**Pass Criteria:** Correct voltage on each channel

---

## Category 6: Integration Tests

### Test 6.1: Full Display Cycle
**Objective:** Verify complete O/P → I/P cycle

**Expected Sequence:**
1. Display "-OP" for 1000ms
2. Display O/P voltage for 3000ms
3. Display "-IP" for 1000ms
4. Display I/P voltage for 3000ms
5. Repeat

**Timing Tolerance:** ±5%

**Pass Criteria:** Correct sequence and timing

---

### Test 6.2: Calibration During Display Cycle
**Objective:** Verify cal mode extends display time

**Test:**
1. Wait for O/P voltage display
2. Press INC at t=2500ms
3. Wait 2000ms
4. Expected: Still showing O/P voltage at t=4500ms

**Pass Criteria:** Display extended by cal timeout

---

### Test 6.3: Long-Term Stability
**Objective:** Verify system runs continuously

**Test:**
- Run for 24 hours
- Monitor for crashes, glitches, or drift

**Expected:**
- No crashes
- Continuous operation
- Stable readings

**Pass Criteria:** 24hr continuous operation

---

### Test 6.4: Temperature Variation
**Objective:** Verify operation across temperature range

**Test Temperatures:**
- 0°C (cold)
- 25°C (room temp)
- 50°C (warm)
- 70°C (max operating temp)

**Expected:** Readings within ±2% across temperature range

**Pass Criteria:** Functional at all temperatures

---

### Test 6.5: VDD Brownout
**Objective:** Verify behavior during power fluctuations

**Test:**
- Reduce VDD to 4.0V briefly
- Return to 5.0V

**Expected:**
- System may reset (acceptable)
- OR continues operation with VDD compensation
- No permanent damage or corruption

**Pass Criteria:** Graceful degradation, recovery on power restore

---

## Category 7: Performance Tests

### Test 7.1: ADC Conversion Time
**Objective:** Measure actual conversion time

**Expected:** 20-30us per conversion

**Measurement:** Use oscilloscope on ADCS signal

**Pass Criteria:** Within expected range

---

### Test 7.2: Display Update Rate
**Objective:** Verify 100ms update rate

**Expected:** Voltage reading updates every 100ms ±10ms

**Pass Criteria:** Consistent update timing

---

### Test 7.3: Button Response Time
**Objective:** Measure calibration response latency

**Expected:**
- Debounce delay: 20ms
- Display update: <100ms
- Total: <150ms from press to display change

**Pass Criteria:** User perceives immediate response

---

### Test 7.4: CPU Utilization
**Objective:** Estimate CPU usage

**Components:**
- Display refresh: ~60% (continuous)
- ADC reading: ~10% (periodic)
- Button scanning: ~5% (periodic)
- Idle: ~25%

**Note:** These are estimates, measure with profiler if available

---

## Test Execution Checklist

### Pre-Test Setup
- [ ] Compile code with no warnings
- [ ] Flash to target hardware
- [ ] Verify VDD = 5.0V ±0.1V
- [ ] Calibrate multimeter
- [ ] Prepare test equipment
- [ ] Document initial conditions

### During Testing
- [ ] Record all results in test log
- [ ] Take photos/screenshots of failures
- [ ] Note any unexpected behavior
- [ ] Monitor power consumption
- [ ] Check for thermal issues

### Post-Test
- [ ] Analyze results
- [ ] Document failures
- [ ] Create bug reports for issues
- [ ] Update code as needed
- [ ] Retest failed scenarios
- [ ] Final acceptance test

---

## Pass/Fail Criteria

### Overall Pass Requirements:
- ✅ All Category 1 (ADC) tests pass
- ✅ All Category 2 (Voltage Calc) tests pass
- ✅ All Category 3 (Display) tests pass
- ✅ All Category 4 (Buttons) tests pass
- ✅ All Category 5 (Edge Cases) tests pass
- ✅ 90% of Category 6 (Integration) tests pass
- ✅ 80% of Category 7 (Performance) tests pass

### Critical Failures (Must Fix):
- ADC timeout not working
- Voltage calculation overflow
- Display inversion incorrect
- Button debounce failure
- System crashes or hangs

### Non-Critical Issues (Can defer):
- Display timing slightly off
- Performance not optimal
- Temperature drift beyond spec

---

## Test Documentation

### Required Logs:
1. **Test Execution Log** - Record of all tests run
2. **Failure Analysis Log** - Details of any failures
3. **Configuration Log** - Hardware setup, compiler settings
4. **Results Summary** - Overall pass/fail status

### Test Report Template:

```
Test ID: [e.g., 2.1]
Test Name: [e.g., Voltage Calculation Accuracy]
Date: [YYYY-MM-DD]
Tester: [Name]
Environment: [Hardware version, compiler version]

Test Steps:
1. [Step 1]
2. [Step 2]
...

Expected Results:
[What should happen]

Actual Results:
[What actually happened]

Status: [PASS / FAIL / BLOCKED]
Notes: [Any observations]
Attachments: [Photos, logs, etc.]
```

---

## Automated Testing (Future)

### Simulation Tests:
- Create PC-based simulator
- Mock hardware registers
- Unit test individual functions

### Hardware-in-Loop:
- Automated voltage source control
- Automated button actuation
- Automated display reading (camera)

---

## Regression Testing

After any code changes, re-run:
- All failed tests from previous iteration
- All Category 1 and 2 tests (critical path)
- Smoke test: Basic functionality check

---

## Appendix A: Test Equipment Calibration

### Multimeter Calibration:
- Verify against known reference voltage
- Check at 0V, 2.5V, 5V points
- Record calibration date

### Oscilloscope Calibration:
- Verify probe compensation
- Check time base accuracy
- Verify trigger levels

### Power Supply Calibration:
- Verify output voltage accuracy
- Check load regulation
- Verify ripple < 50mV

---

## Appendix B: Known Limitations

1. **VDD Range:** Code tested for 4.5V - 5.5V only
2. **Temperature:** Not compensated for temperature drift
3. **Input Protection:** No overvoltage protection in software
4. **Calibration:** Not saved to EEPROM (lost on power cycle)

---

**Test Plan Version History:**
- v1.0 (2025-11-17): Initial test plan creation

**Approvals:**
- Developer: _______________ Date: ___________
- Reviewer: _______________ Date: ___________
- Tester: _______________ Date: ___________
