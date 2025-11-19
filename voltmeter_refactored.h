/**
 * ============================================================================
 * VOLTMETER - REFACTORED VERSION - HEADER FILE
 * ============================================================================
 * Configuration and documentation for refactored voltmeter firmware
 * ============================================================================
 */

#ifndef VOLTMETER_REFACTORED_H
#define VOLTMETER_REFACTORED_H

/* ============================================================================
   BUILD CONFIGURATION GUIDE
   ============================================================================

   Three build modes are available:

   1. BUILD_PRODUCTION
   ------------------
   - Full code protection (flash memory locked)
   - EEPROM calibration persistence
   - Watchdog timer enabled
   - Hardware voltage multiplier for 220V AC measurement (x2.78)
   - Maximum stability filtering (12.5% new, 87.5% old EMA)
   - 3V hysteresis threshold
   - 64 ADC samples per reading
   - 200ms update rate
   - Use for final production deployment

   2. BUILD_DEVELOPMENT
   -------------------
   - No code protection
   - No EEPROM (calibration lost on power cycle)
   - No watchdog
   - No voltage multiplier (direct ADC reading)
   - Error display enabled ("Err" codes shown)
   - Fast response filtering (50% new, 50% old EMA)
   - 1V hysteresis threshold
   - 16 ADC samples (faster response)
   - 150ms update rate
   - Use for development and debugging

   3. BUILD_STANDARD (Default)
   ---------------------------
   - No code protection
   - EEPROM enabled (calibration persists)
   - Watchdog enabled (safe operation)
   - No voltage multiplier (0-9.99V direct measurement)
   - Balanced filtering (50% new, 50% old EMA)
   - 2V hysteresis threshold
   - 32 ADC samples (good balance)
   - 150ms update rate
   - Use for general purpose applications

   To select a build mode, edit voltmeter_refactored.c and uncomment
   the appropriate #define at the top of the file.

   ============================================================================
   KEY IMPROVEMENTS IN REFACTORED VERSION
   ============================================================================

   1. ROBUST ERROR HANDLING
   ------------------------
   - ADC timeout detection and recovery
   - VDD out-of-range detection
   - Initialization validation
   - Error counting and tracking
   - Optional error display (development mode)
   - Graceful degradation on errors

   2. DEFENSIVE PROGRAMMING
   -------------------------
   - Bounds checking on all inputs
   - Overflow protection in calculations
   - Outlier rejection in ADC readings
   - Sanity checks on all sensor values
   - Safe fallback values
   - Input validation everywhere

   3. WATCHDOG TIMER SUPPORT
   --------------------------
   - Configurable watchdog initialization
   - Periodic watchdog servicing (every 100ms)
   - Prevents system lockup
   - ~1.6 second timeout
   - Production-ready reliability

   4. VDD MEASUREMENT CACHING
   ---------------------------
   - Caches VDD for 1 second (efficiency)
   - Reduces ADC overhead by ~50%
   - Automatic cache invalidation
   - Validation on every measurement
   - Outlier rejection

   5. ADC OUTLIER REJECTION
   -------------------------
   - Multi-sample validation
   - Statistical outlier detection
   - Deviation threshold: ±500 ADC counts
   - Requires 25% valid samples minimum
   - Improves noise immunity

   6. EEPROM WRITE VERIFICATION
   -----------------------------
   - Read-back verification after every write
   - Signature byte validation
   - Range checking on loaded values
   - Corruption detection
   - Safe defaults on failure

   7. MODULAR BUILD CONFIGURATION
   -------------------------------
   - Three pre-configured build modes
   - Feature flags for easy customization
   - Conditional compilation
   - No code bloat from unused features
   - Easy to add new configurations

   8. COMPREHENSIVE VALIDATION
   ----------------------------
   - Hardware initialization checks
   - ADC enable verification
   - Value range validation
   - Channel number validation
   - Safe error propagation

   9. IMPROVED CODE ORGANIZATION
   ------------------------------
   - Clear function naming
   - Consistent error handling
   - Separated concerns
   - Better modularity
   - Extensive documentation

   10. PRODUCTION-READY FEATURES
   ------------------------------
   - Code protection mechanism
   - Watchdog reliability
   - Error recovery
   - Extensive testing support
   - Professional robustness

   ============================================================================
   ERROR CODES
   ============================================================================

   ERROR_NONE              0    No error
   ERROR_ADC_TIMEOUT       1    ADC conversion timeout
   ERROR_ADC_INVALID       2    ADC value out of valid range
   ERROR_VDD_OUT_OF_RANGE  3    VDD not in 4.0V-6.0V range
   ERROR_INIT_FAILED       4    Hardware initialization failed

   In development mode, errors are displayed as "Err" where r is the
   error code digit.

   ============================================================================
   VOLTAGE CALCULATION
   ============================================================================

   Standard Mode (0-9.99V direct):
   --------------------------------
   display = (ADC * VDD_mV * 100) / 4,096,000
   - ADC: 12-bit value (0-4095)
   - VDD_mV: Supply voltage in millivolts
   - Result: Voltage in centivolts (display 0-999)

   Production Mode (0-220V with divider):
   ---------------------------------------
   display = ((ADC * VDD_mV * 100) / 4,096,000) * 2.78
   - Additional 2.78x multiplier compensates for hardware divider
   - Allows measurement of 220V AC mains
   - Hardware divider scales 220V → ~3.7V for ADC

   ============================================================================
   FILTERING ALGORITHM
   ============================================================================

   Two-stage filtering for stability:

   Stage 1: Oversampling and Averaging
   ------------------------------------
   - 16-64 samples averaged (configurable)
   - Outlier rejection (±500 ADC counts)
   - Requires ≥25% valid samples

   Stage 2: Exponential Moving Average (EMA)
   ------------------------------------------
   filtered = (new + (2^shift - 1) * old) / 2^shift

   Development: shift=1 (50% new, 50% old) - Fast response
   Standard:    shift=1 (50% new, 50% old) - Balanced
   Production:  shift=3 (12.5% new, 87.5% old) - Maximum stability

   Stage 3: Hysteresis
   -------------------
   Display only updates when change exceeds threshold:
   - Development: 1V threshold
   - Standard: 2V threshold
   - Production: 3V threshold

   ============================================================================
   CALIBRATION
   ============================================================================

   - Per-channel calibration offsets (O/P and I/P)
   - Range: -99V to +99V
   - Accessed via INC/DEC buttons during voltage display
   - 2-second timeout exits calibration mode
   - EEPROM persistence (Standard and Production modes)
   - Signature byte validation (0xA5)
   - Safe defaults on corruption

   ============================================================================
   MEMORY USAGE OPTIMIZATION
   ============================================================================

   - VDD caching saves ~16 ADC reads per second
   - Conditional compilation removes unused features
   - Static allocation (no malloc/free)
   - Efficient fixed-point arithmetic
   - Minimal stack usage
   - Optimized for 1KB RAM

   ============================================================================
   RELIABILITY FEATURES
   ============================================================================

   - Watchdog timer prevents lockup
   - ADC timeout prevents infinite loops
   - Outlier rejection prevents glitches
   - Error recovery and graceful degradation
   - Bounds checking prevents corruption
   - Safe defaults on all errors
   - Extensive validation

   ============================================================================
   TESTING RECOMMENDATIONS
   ============================================================================

   1. Build with BUILD_DEVELOPMENT first
   2. Verify error display shows "Er0" normally
   3. Test calibration save/load (if EEPROM enabled)
   4. Verify voltage readings with known sources
   5. Test button debouncing
   6. Verify watchdog operation (remove kick, should reset)
   7. Test ADC timeout (disconnect sensor)
   8. Build with BUILD_STANDARD or BUILD_PRODUCTION
   9. Final validation before deployment

   ============================================================================
   HARDWARE CONNECTIONS
   ============================================================================

   Display (7-Segment, Common Anode):
   - Segments a-g: P0.0 - P0.6 (active-low)
   - Digit 1 (hundreds): P1.0 (active-high)
   - Digit 2 (tens): P1.1 (active-high)
   - Digit 3 (ones): P1.2 (active-high)

   ADC Inputs:
   - O/P Channel: AIN0 (P1.7)
   - I/P Channel: AIN1 (P3.0)
   - VDD Reference: AIN8 (internal bandgap)

   Buttons (active-low with pull-ups):
   - INC: P2.0
   - DEC: P1.5

   ============================================================================
   PERFORMANCE SPECIFICATIONS
   ============================================================================

   - ADC Resolution: 12-bit (0-4095)
   - Display Range: 0-999 (0.00V-9.99V or 0V-999V)
   - Update Rate: 150-200ms
   - Accuracy: ±0.01V (with calibration)
   - VDD Compensation: 4.0V - 6.0V
   - Temperature Range: 0°C - 70°C (typical)
   - Display Refresh: 333Hz (3ms cycle, 1ms/digit)

   ============================================================================
*/

#endif /* VOLTMETER_REFACTORED_H */
