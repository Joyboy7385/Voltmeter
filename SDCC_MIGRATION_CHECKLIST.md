# SDCC MIGRATION CHECKLIST
## Complete Guide to Port Voltmeter Code from Keil to SDCC

**Project:** MS51 Voltmeter
**Original Compiler:** Keil C51
**Target Compiler:** SDCC (Small Device C Compiler)
**Date Started:** 2025-11-19

---

## ⚠️ CRITICAL WARNINGS

- [ ] **BACKUP ENTIRE PROJECT** before starting
- [ ] **DO NOT DELETE** original Keil files
- [ ] **TEST EACH STEP** before proceeding to next
- [ ] **VERIFY COMPILATION** after each major change
- [ ] **DO NOT FLASH** to hardware until ALL checklists complete

---

## 📋 PHASE 1: PRE-MIGRATION PREPARATION

### 1.1 Environment Setup
- [ ] Install SDCC compiler (version 4.0+)
  ```bash
  # Verify installation:
  sdcc --version
  # Expected: SDCC 4.x.x or higher
  ```
- [ ] Verify SDCC installed correctly: `which sdcc`
- [ ] Install `make` utility: `which make`
- [ ] Install `stcgal` or equivalent flash programmer for MS51
- [ ] Create backup of entire project directory
- [ ] Verify backup integrity

### 1.2 Documentation Review
- [ ] Read MS51FB9AE datasheet (focus on SFR addresses)
- [ ] Read SDCC manual section on 8051 support
- [ ] Understand SDCC memory models (small, medium, large)
- [ ] Review SDCC interrupt syntax differences
- [ ] Note SDCC limitations vs Keil

### 1.3 File Organization
- [ ] Create `sdcc/` subdirectory in project
- [ ] Create `sdcc/include/` for headers
- [ ] Create `sdcc/src/` for source files
- [ ] Create `sdcc/build/` for compiled outputs
- [ ] Create `sdcc/backup/` for safety

---

## 📋 PHASE 2: HEADER FILE CREATION

### 2.1 Create ms51_sdcc.h (SFR Definitions)
- [ ] Define all SFR registers with `__sfr __at (address)`
- [ ] Verify each SFR address against MS51 datasheet
- [ ] Define all bit-addressable SFRs
- [ ] Define all sbit variables with `__sbit __at (address)`
- [ ] Verify bit addresses (formula: byte_addr*8 + bit_position)
- [ ] Test compile: `sdcc -c -mmcs51 test.c` with header included

**SFRs to Define:**
- [ ] AUXR0 (0x8E)
- [ ] ADCCON0 (0xE7)
- [ ] ADCCON1 (0xE1)
- [ ] ADCDLY (0xE6)
- [ ] ADCRH (0xC3)
- [ ] ADCRL (0xC2)
- [ ] ADCF bit (ADCCON0^7 = 0xEF)
- [ ] ADCS bit (ADCCON0^6 = 0xEE)
- [ ] CHPCON (check address)
- [ ] IAPEN bit (check address)
- [ ] IAPTRG (check address)
- [ ] IAPGO bit (check address)
- [ ] IAPCN (check address)
- [ ] IAPAL (check address)
- [ ] IAPAH (check address)
- [ ] IAPFD (check address)
- [ ] IAPUEN (check address)
- [ ] WDCON (check address)
- [ ] WDTR bit (check address)
- [ ] WDCLR bit (check address)
- [ ] AINDIDS (check address)
- [ ] P0M1, P0M2 (check addresses)
- [ ] P1M1, P1M2 (check addresses)
- [ ] P3M1, P3M2 (check addresses)
- [ ] SFRS (check address)
- [ ] TA (Timed Access) (check address)

### 2.2 Create sfr_macro_sdcc.h (Macro Definitions)
- [ ] Convert all `set_XXX` macros to direct bit assignments
- [ ] Convert all `clr_XXX` macros to direct bit assignments
- [ ] Convert all pin mode macros to register operations
- [ ] Test each macro definition independently
- [ ] Verify macro expansion with `-E` flag

**Critical Macros:**
- [ ] set_ADCS → `ADCS = 1`
- [ ] clr_ADCF → `ADCF = 0`
- [ ] set_CHPCON_IAPEN → `IAPEN = 1`
- [ ] clr_CHPCON_IAPEN → `IAPEN = 0`
- [ ] set_IAPTRG_IAPGO → `IAPGO = 1`
- [ ] set_WDCON_WDTR → `WDTR = 1`
- [ ] set_WDCON_WDCLR → `WDCLR = 1`
- [ ] P00_PUSHPULL_MODE → register operations
- [ ] P17_Input_Mode → register operations
- [ ] P30_Input_Mode → register operations

### 2.3 Create function_define_sdcc.h
- [ ] Define HIBYTE macro
- [ ] Define LOBYTE macro
- [ ] Define SET_BITx constants
- [ ] Define CLR_BITx constants
- [ ] Test macro correctness with sample code

### 2.4 Header File Verification
- [ ] Compile empty main() with all headers: NO ERRORS
- [ ] Compile with SFR access test: NO WARNINGS
- [ ] Compile with bit operations: NO WARNINGS
- [ ] Check for duplicate definitions: NONE
- [ ] Check for missing definitions: NONE

---

## 📋 PHASE 3: DELAY FUNCTION PORTING

### 3.1 Create delay_sdcc.c
- [ ] Port Timer0_Delay function
- [ ] Replace Keil macros with SDCC bit operations
- [ ] Verify timer calculation formulas
- [ ] Test compile delay_sdcc.c
- [ ] Verify no warnings during compilation

### 3.2 Delay Function Testing
- [ ] Test with 1ms delay parameter
- [ ] Test with 100us delay parameter
- [ ] Verify timer overflow logic
- [ ] Check for infinite loops
- [ ] Verify all paths return properly

---

## 📋 PHASE 4: MAIN CODE CONVERSION

### 4.1 Create voltmeter_sdcc.c
- [ ] Copy voltmeter_refactored.c as base
- [ ] Replace header includes with SDCC versions
- [ ] Replace all `set_XXX` macros
- [ ] Replace all `clr_XXX` macros
- [ ] Replace all pin mode macros
- [ ] Replace Timer0_Delay calls
- [ ] Update BIT_TMP usage if needed
- [ ] Check all function signatures

### 4.2 Critical Code Sections Review
- [ ] Watchdog initialization (lines 177-190)
- [ ] EEPROM write/read (lines 249-291)
- [ ] Code protection (lines 339-370) - **DISABLE for testing**
- [ ] ADC initialization (lines 427-450)
- [ ] ADC reading (lines 458-503)
- [ ] I/O initialization (lines 919-944)

### 4.3 Build Configuration
- [ ] Set ENABLE_CODE_PROTECTION to 0 for testing
- [ ] Keep ENABLE_WATCHDOG = 0 initially
- [ ] Set BUILD_STANDARD or BUILD_DEVELOPMENT
- [ ] Verify all #define values are correct

---

## 📋 PHASE 5: COMPILATION

### 5.1 Create Makefile
- [ ] Set SDCC compiler path
- [ ] Set correct MCU target (mcs51)
- [ ] Set memory model (--model-small recommended)
- [ ] Set optimization level (-O2 or -O3)
- [ ] Add all source files to build
- [ ] Configure output format (.hex for flashing)
- [ ] Test Makefile syntax: `make -n`

### 5.2 First Compilation Attempt
- [ ] Run `make clean`
- [ ] Run `make`
- [ ] **EXPECT ERRORS** - this is normal!
- [ ] Document all error messages
- [ ] Fix errors one by one
- [ ] Re-compile after each fix

### 5.3 Common SDCC Errors to Fix
- [ ] Undefined SFR errors → check ms51_sdcc.h
- [ ] Undefined bit errors → check bit addresses
- [ ] Macro expansion errors → check sfr_macro_sdcc.h
- [ ] Type mismatch warnings → add explicit casts
- [ ] Unreachable code warnings → review logic
- [ ] Stack overflow warnings → reduce local variables

### 5.4 Compilation Success Criteria
- [ ] ZERO errors
- [ ] ZERO warnings (aim for this)
- [ ] .hex file generated
- [ ] .ihx file generated
- [ ] .map file generated
- [ ] Memory usage report looks reasonable

---

## 📋 PHASE 6: CODE SIZE & MEMORY VERIFICATION

### 6.1 Memory Analysis
- [ ] Check code size < 16KB (MS51 APROM limit)
- [ ] Check RAM usage < 256 bytes (MS51 IRAM limit)
- [ ] Check XRAM usage if used
- [ ] Review .map file for memory layout
- [ ] Verify no stack overflow risk

### 6.2 Code Optimization (if needed)
- [ ] Try different optimization levels
- [ ] Remove debug code if size critical
- [ ] Use `--stack-auto` carefully (may increase code size)
- [ ] Consider `--model-small` vs `--model-medium`

---

## 📋 PHASE 7: PRE-FLASH VERIFICATION

### 7.1 Static Analysis
- [ ] Review disassembly (.asm file)
- [ ] Verify interrupt vectors if used
- [ ] Check initialization code
- [ ] Verify watchdog disabled for testing
- [ ] Check code protection disabled

### 7.2 Logic Verification
- [ ] Trace through main() logic
- [ ] Verify ADC initialization order
- [ ] Check display multiplexing timing
- [ ] Verify button debounce logic
- [ ] Check EEPROM access if enabled

### 7.3 Safety Checks
- [ ] Code protection DISABLED
- [ ] Watchdog DISABLED for first test
- [ ] No infinite loops in init code
- [ ] All pins initialized properly
- [ ] No uninitialized variables

---

## 📋 PHASE 8: HARDWARE TESTING

### 8.1 First Flash (CRITICAL - READ CAREFULLY)
- [ ] **BACKUP current working firmware** from chip
- [ ] Connect programmer to MS51
- [ ] Verify power supply (5V ±0.25V)
- [ ] Flash .hex file using stcgal/programmer
- [ ] **DO NOT POWER CYCLE YET**
- [ ] Verify flash successful (no errors)
- [ ] Note flash time and size

### 8.2 Power-On Test #1: Basic Boot
- [ ] Power cycle the board
- [ ] **OBSERVE**: Does display light up?
- [ ] **OBSERVE**: Any segments visible?
- [ ] **OBSERVE**: Any digit selection?
- [ ] **If nothing**: IMMEDIATELY reflash backup
- [ ] **If garbled**: Check clock/timing settings
- [ ] **If working**: Proceed to next test

### 8.3 Power-On Test #2: Display Multiplexing
- [ ] Verify all three digits can light up
- [ ] Check for ghosting between digits
- [ ] Verify brightness acceptable
- [ ] Check for flicker (should be minimal)
- [ ] Measure multiplex timing with scope if available

### 8.4 Test #3: Voltage Reading
- [ ] Apply known voltage to O/P channel (start with 0V)
- [ ] Wait for "O-P" label to appear
- [ ] **EXPECTED**: Shows voltage value
- [ ] **VERIFY**: Reading is stable
- [ ] Apply 3.3V reference → check reading
- [ ] Apply 5.0V reference → check reading
- [ ] **If readings wrong**: Check ADC init code

### 8.5 Test #4: Channel Switching
- [ ] Verify "O-P" label displays correctly
- [ ] Wait 3 seconds for auto-switch
- [ ] Verify "I-P" label displays correctly
- [ ] Verify voltage reading updates
- [ ] Check cycle repeats properly

### 8.6 Test #5: Button Functionality
- [ ] Press INC button during voltage display
- [ ] **EXPECTED**: Calibration mode activates
- [ ] Verify value increments
- [ ] Release button
- [ ] Verify timeout (2 seconds)
- [ ] Repeat for DEC button

### 8.7 Test #6: EEPROM Calibration (if enabled)
- [ ] Set calibration offset (e.g., +5)
- [ ] Wait for timeout/save
- [ ] Power cycle the board
- [ ] **EXPECTED**: Calibration persists
- [ ] Verify offset still applied
- [ ] Reset to zero
- [ ] Power cycle again to verify

### 8.8 Test #7: Watchdog (enable only after all tests pass)
- [ ] Enable ENABLE_WATCHDOG = 1
- [ ] Recompile and flash
- [ ] **VERIFY**: Board still works normally
- [ ] Introduce deliberate infinite loop
- [ ] **EXPECTED**: Watchdog resets chip
- [ ] Remove infinite loop
- [ ] Verify normal operation resumes

### 8.9 Stress Testing
- [ ] Run for 1 hour continuously
- [ ] Monitor for any glitches
- [ ] Check for display anomalies
- [ ] Verify no unexpected resets
- [ ] Monitor temperature (chip shouldn't be hot)

---

## 📋 PHASE 9: KNOWN ISSUES & TROUBLESHOOTING

### 9.1 No Display After Flash
**Symptoms:** All segments off, no response
**Likely Causes:**
- [ ] Check clock configuration (SDCC may use different defaults)
- [ ] Verify IO_Init() executed
- [ ] Check P0 and P1 pin configurations
- [ ] Verify Timer0_Delay works correctly
- [ ] Oscilloscope P0 to check segment signals

**Fix Steps:**
1. Add LED blink test in main() before display code
2. Simplify to single digit display
3. Check with scope for activity

### 9.2 Garbled Display
**Symptoms:** Random segments lit, flickering
**Likely Causes:**
- [ ] Timer0_Delay timing incorrect
- [ ] Display multiplex too slow/fast
- [ ] Ghosting between digits
- [ ] Incorrect CA_OUTPUT calculation

**Fix Steps:**
1. Increase/decrease delay timing
2. Add longer blank period between digits
3. Check digit turn-off code

### 9.3 Wrong Voltage Readings
**Symptoms:** Values don't match input
**Likely Causes:**
- [ ] ADC not initialized properly
- [ ] VDD measurement incorrect
- [ ] Calculation overflow in voltage conversion
- [ ] Wrong ADC reference

**Fix Steps:**
1. Read raw ADC values first (add debug display)
2. Verify VDD calculation
3. Check for integer overflow in math
4. Verify ADC clock divider setting

### 9.4 Buttons Don't Work
**Symptoms:** No response to button press
**Likely Causes:**
- [ ] P2.0 RESET not disabled (AUXR0 setting)
- [ ] Pin mode incorrect (should be input)
- [ ] Debounce logic error

**Fix Steps:**
1. Check AUXR0 &= ~0x04 executed
2. Verify BTN_INC and BTN_DEC sbit definitions
3. Add raw button state display

### 9.5 EEPROM Not Saving
**Symptoms:** Calibration lost after power cycle
**Likely Causes:**
- [ ] EEPROM write not completing
- [ ] IAPEN not set correctly
- [ ] Wrong EEPROM command codes
- [ ] Timing issue with TA (timed access)

**Fix Steps:**
1. Add verification after EEPROM write
2. Check CHPCON settings
3. Verify TA = 0xAA; TA = 0x55 sequence

### 9.6 Random Resets
**Symptoms:** Board resets unexpectedly
**Likely Causes:**
- [ ] Watchdog enabled and not being kicked
- [ ] Stack overflow
- [ ] Invalid pointer access
- [ ] Power supply noise

**Fix Steps:**
1. Disable watchdog completely
2. Reduce stack usage (fewer locals)
3. Add stack usage reporting
4. Check power supply quality

---

## 📋 PHASE 10: FINAL VALIDATION

### 10.1 Full Regression Testing
- [ ] All voltage ranges (0-999)
- [ ] Both channels (O/P and I/P)
- [ ] Button calibration on both channels
- [ ] EEPROM save/load on both channels
- [ ] Power cycle 10 times
- [ ] 24-hour continuous run
- [ ] Temperature range testing if applicable

### 10.2 Code Quality Check
- [ ] No compiler warnings
- [ ] No magic numbers (use #defines)
- [ ] All functions have comments
- [ ] No commented-out debug code
- [ ] Consistent formatting

### 10.3 Documentation Update
- [ ] Update README with SDCC build instructions
- [ ] Document known SDCC-specific issues
- [ ] Add Makefile usage guide
- [ ] Note any behavioral differences from Keil version

---

## 📋 PHASE 11: OPTIONAL PRODUCTION FEATURES

### 11.1 Enable Code Protection (ONLY after ALL tests pass)
- [ ] Set ENABLE_CODE_PROTECTION = 1
- [ ] **WARNING**: This will LOCK the chip!
- [ ] Test on spare chip first
- [ ] Verify you have backup firmware
- [ ] Flash and test
- [ ] **CANNOT REVERSE** - chip is locked

### 11.2 Enable All Production Features
- [ ] ENABLE_WATCHDOG = 1
- [ ] ENABLE_VOLTAGE_MULTIPLIER (if using 220V)
- [ ] Optimize for size/speed as needed
- [ ] Final burn-in testing

---

## ✅ SIGN-OFF CHECKLIST

**I certify that:**
- [ ] All Phase 1-8 checklists are 100% complete
- [ ] Hardware testing passed all criteria
- [ ] No known bugs or issues remain
- [ ] Code is backed up in multiple locations
- [ ] Documentation is complete and accurate
- [ ] Ready for production use

**Signed:** ________________
**Date:** ________________

---

## 📞 EMERGENCY RECOVERY

**If chip becomes unresponsive:**
1. Attempt to reflash with backup firmware
2. Try ISP mode entry (check MS51 datasheet for procedure)
3. Use Nuvoton ICP programmer if available
4. If code protection was enabled: **Chip may be bricked**
5. Have spare MS51 chips available!

**Important Contact Info:**
- SDCC Mailing List: sdcc-user@lists.sourceforge.net
- Nuvoton Support: [insert contact]
- Project Repository: [insert URL]

---

**END OF CHECKLIST**
