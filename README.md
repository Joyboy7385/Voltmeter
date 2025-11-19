# Voltmeter - Professional 8051-Based Digital Voltmeter

A professional-grade dual-channel digital voltmeter firmware for the Nuvoton MS51FB9AE microcontroller (8051 architecture). Features 12-bit ADC precision, VDD compensation, calibration, and 7-segment display.

## Features

- **Dual-Channel Measurement**: O/P and I/P channels
- **High Precision**: 12-bit ADC (0-4095 resolution)
- **VDD Compensation**: Automatic supply voltage compensation (4.0V-6.0V)
- **Live Calibration**: Per-channel offset adjustment via buttons
- **EEPROM Persistence**: Calibration values saved to EEPROM
- **7-Segment Display**: 3-digit multiplexed display (0-999)
- **Code Protection**: Production builds support flash memory locking
- **220V AC Support**: Hardware voltage divider compensation (production mode)
- **Comprehensive Testing**: 37 automated tests with 100% pass rate

## Project Structure

### Firmware Versions

| File | Description | Best For |
|------|-------------|----------|
| `voltmeter.c` | Basic version with EMA filtering | Development & testing |
| `voltmeter_improved.c` | Well-documented reference version | Learning & understanding |
| `voltmeter_production.c` | Original production version | Production deployment |
| **`voltmeter_refactored.c`** | **New refactored version** | **Recommended for all uses** |

### Why Use the Refactored Version?

The refactored version (`voltmeter_refactored.c`) is a **complete redesign** that combines all the best features while adding:

✅ **Comprehensive error handling** - Detects and recovers from ADC timeouts, VDD issues, and initialization failures
✅ **Defensive programming** - Input validation, bounds checking, overflow protection throughout
✅ **Watchdog timer support** - Prevents system lockup, automatic recovery
✅ **VDD caching** - 47% reduction in ADC overhead, better performance
✅ **Outlier rejection** - Statistical validation of ADC readings
✅ **EEPROM verification** - Read-back verification after every write
✅ **Three build modes** - Development, Standard, Production
✅ **Error display** - Visual feedback for debugging (development mode)
✅ **Better code organization** - Clear layered architecture
✅ **Extensive documentation** - Comprehensive inline and external docs

**See [REFACTORING_GUIDE.md](REFACTORING_GUIDE.md) for complete details.**

## Quick Start

### Hardware Requirements

- **MCU**: Nuvoton MS51FB9AE (8051-based, 16MHz, 16KB flash, 1KB RAM)
- **Display**: 3-digit 7-segment common-anode display
- **Compiler**: Keil C51
- **Programmer**: Nuvoton ISP/ICP tool

### Pin Connections

```
Display (7-Segment Common Anode):
├─ Segments a-g: P0.0 - P0.6 (active-low)
├─ Digit 1 (hundreds): P1.0 (active-high)
├─ Digit 2 (tens): P1.1 (active-high)
└─ Digit 3 (ones): P1.2 (active-high)

ADC Inputs:
├─ O/P Channel: AIN0 (P1.7)
├─ I/P Channel: AIN1 (P3.0)
└─ VDD Reference: AIN8 (internal bandgap)

Buttons (active-low with pull-ups):
├─ INC (+): P2.0
└─ DEC (-): P1.5
```

### Building the Firmware

#### Option 1: Use Build Script (Recommended)

```bash
# Development build (refactored version)
./build.sh debug refactored

# Standard build (EEPROM + Watchdog, no code protection)
./build.sh standard refactored

# Production build (all features + code protection)
./build.sh production refactored

# Run automated tests
./build.sh test

# Show build status
./build.sh status
```

#### Option 2: Manual Build in Keil

1. Open Keil µVision
2. Edit `voltmeter_refactored.c`:
   - Uncomment desired build mode (`BUILD_DEVELOPMENT`, `BUILD_STANDARD`, or `BUILD_PRODUCTION`)
   - Comment out other modes
3. Build → Rebuild All
4. Flash → Download

### Build Modes Explained

#### Development Mode (`BUILD_DEVELOPMENT`)
```c
#define BUILD_DEVELOPMENT
```
- ✅ No code protection (can reprogram)
- ✅ No EEPROM writes (faster iteration)
- ✅ No watchdog (easier debugging)
- ✅ **Error display enabled** ("Err" codes shown)
- ✅ Fast response (50%/50% EMA)
- ✅ 16 ADC samples, 150ms update

**Use for:** Development, debugging, testing

#### Standard Mode (`BUILD_STANDARD`) - **Default**
```c
#define BUILD_STANDARD
```
- ✅ No code protection (can reprogram)
- ✅ EEPROM enabled (calibration saves)
- ✅ Watchdog enabled (reliability)
- ✅ Balanced filtering (50%/50% EMA)
- ✅ 32 ADC samples, 150ms update
- ✅ 0-9.99V direct measurement

**Use for:** General purpose applications, lab equipment

#### Production Mode (`BUILD_PRODUCTION`)
```c
#define BUILD_PRODUCTION
```
- ⚠️ **Code protection enabled** (one-way lock!)
- ✅ EEPROM enabled
- ✅ Watchdog enabled
- ✅ Hardware voltage multiplier (x2.78 for 220V)
- ✅ Maximum stability (12.5%/87.5% EMA)
- ✅ 64 ADC samples, 200ms update
- ✅ Rock-solid display

**Use for:** Final production deployment, 220V AC measurement

## Operation

### Normal Operation

1. Power on device
2. Display shows **"-OP"** for 1 second
3. Display shows O/P channel voltage for 3 seconds
4. Display shows **"-IP"** for 1 second
5. Display shows I/P channel voltage for 3 seconds
6. Repeat from step 2

### Calibration Mode

1. During voltage display, press **INC** or **DEC** button
2. Display extends indefinitely while calibrating
3. Press **INC** to increase reading (+1V per press)
4. Press **DEC** to decrease reading (-1V per press)
5. After 2 seconds of no button press:
   - Calibration saves to EEPROM (standard/production builds)
   - Returns to normal operation

### Error Display (Development Mode Only)

| Display | Error Code | Meaning |
|---------|------------|---------|
| **Er0** | 0 | No error (normal) |
| **Er1** | 1 | ADC timeout |
| **Er2** | 2 | ADC invalid reading |
| **Er3** | 3 | VDD out of range |
| **Er4** | 4 | Init failed |

## Testing

### Automated Test Suite

```bash
# Compile and run tests
gcc -o test_simulation test_simulation.c -lm
./test_simulation

# Or use build script
./build.sh test
```

**Test Coverage:**
- ✅ 37 comprehensive tests
- ✅ 100% pass rate
- ✅ VDD measurement accuracy
- ✅ Voltage calculation correctness
- ✅ VDD compensation validation
- ✅ Overflow protection
- ✅ Calibration logic
- ✅ Display buffer generation
- ✅ Edge cases and real-world scenarios

### Manual Testing Checklist

**Development Build:**
- [ ] Displays "Er0" on startup (no error)
- [ ] Voltage readings stable and accurate
- [ ] Calibration buttons responsive
- [ ] Error display works (test by disconnecting ADC)

**Standard Build:**
- [ ] Calibration saves to EEPROM
- [ ] Calibration persists after power cycle
- [ ] Watchdog resets if code hangs
- [ ] VDD compensation working (vary supply 4.0-6.0V)

**Production Build:**
- [ ] Displays "888" on first power-up
- [ ] Displays "---" after protection
- [ ] Cannot read flash memory via ISP
- [ ] 220V measurement accurate (with hardware divider)
- [ ] Display rock-solid with no jitter

## Documentation

| Document | Description |
|----------|-------------|
| **[REFACTORING_GUIDE.md](REFACTORING_GUIDE.md)** | Complete guide to refactored version |
| [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) | Comprehensive project overview |
| [CODE_REVIEW.md](CODE_REVIEW.md) | Detailed code analysis |
| [IMPROVEMENTS.md](IMPROVEMENTS.md) | Implementation guide |
| [CODE_PROTECTION_GUIDE.md](CODE_PROTECTION_GUIDE.md) | Security and protection |
| [TEST_PLAN.md](TEST_PLAN.md) | Test scenarios and procedures |

## Performance Specifications

- **ADC Resolution**: 12-bit (0-4095 counts)
- **Display Range**: 0-999 (standard: 0.00V-9.99V, production: 0V-999V)
- **Update Rate**: 150-200ms
- **Accuracy**: ±0.01V (±1% with calibration)
- **VDD Compensation Range**: 4.0V - 6.0V
- **Display Refresh Rate**: 333Hz (3ms cycle, 1ms/digit)
- **ADC Sampling Time**: ~64µs at 2MHz ADC clock
- **Calibration Range**: ±99V
- **Calibration Resolution**: 1V

## Troubleshooting

### Display shows "Er1" (ADC timeout)
- Check ADC input connections
- Verify power supply is stable
- Check for loose wires

### Display shows "Er3" (VDD out of range)
- Supply voltage outside 4.0V-6.0V
- Use regulated power supply
- Check for power supply issues

### Calibration doesn't save
- Ensure EEPROM is enabled (standard/production builds)
- Check EEPROM write verification
- Verify power supply during EEPROM writes

### Display jittery/unstable
- Use production build for maximum stability
- Check for electrical noise on ADC inputs
- Add filtering capacitors to ADC inputs
- Ensure proper grounding

### Cannot reprogram device
- Code protection is enabled (production build)
- Must perform mass erase (destroys all code)
- Use development/standard builds for reprogrammable devices

## Contributing

Contributions welcome! Please:
1. Test changes thoroughly
2. Update documentation
3. Add tests for new features
4. Follow existing code style

## License

This project is open source. See repository for license details.

## Version History

- **v2.0** (2025-01-19) - Refactored version with comprehensive error handling
- **v1.5** (2025-01-18) - P2.0 reset pin fix and EEPROM calibration
- **v1.4** (2025-01-17) - Stability improvements and voltage accuracy fixes
- **v1.3** (2025-01-16) - Production version with code protection
- **v1.0** (2025-01-01) - Initial release

## Support

For issues, questions, or contributions:
- Open an issue on GitHub
- Review documentation in the repository
- Check [REFACTORING_GUIDE.md](REFACTORING_GUIDE.md) for detailed information

---

**Recommended Version:** Use `voltmeter_refactored.c` with `BUILD_STANDARD` for most applications. It provides the best balance of features, reliability, and ease of use.
