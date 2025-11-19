# MS51 Voltmeter - SDCC Port

## 📋 Overview

This is the SDCC (Small Device C Compiler) port of the MS51 Voltmeter project, originally written for Keil C51. This port provides a fully open-source toolchain for building the voltmeter firmware.

**Status:** ✅ Ready for compilation and testing
**Compiler:** SDCC 4.0+ required
**Target MCU:** Nuvoton MS51FB9AE / MS51XB9AE / MS51XB9BE
**Date:** 2025-11-19

---

## ⚠️ CRITICAL: READ BEFORE COMPILING

1. **THIS IS A PORT** - The code has been converted from Keil to SDCC but NOT YET TESTED ON HARDWARE
2. **BACKUP YOUR CHIP** - Always backup existing firmware before flashing
3. **CODE PROTECTION DISABLED** - Default build has code protection OFF for safety
4. **WATCHDOG DISABLED** - Initial build has watchdog OFF for easier debugging
5. **FOLLOW THE CHECKLIST** - Use `SDCC_MIGRATION_CHECKLIST.md` (in parent directory) for safe deployment

---

## 📁 Directory Structure

```
sdcc/
├── include/                    # Header files
│   ├── ms51_sdcc.h            # SFR definitions for MS51
│   ├── sfr_macro_sdcc.h       # SFR access macros
│   └── function_define_sdcc.h # Helper functions and types
├── src/                        # Source files
│   ├── voltmeter_sdcc.c       # Main application
│   └── delay_sdcc.c           # Timer delay functions
├── build/                      # Build output (generated)
│   ├── voltmeter_sdcc.ihx     # Intel HEX (intermediate)
│   ├── voltmeter_sdcc.hex     # Intel HEX (final)
│   ├── voltmeter_sdcc.map     # Memory map
│   └── voltmeter_sdcc.mem     # Memory usage report
├── Makefile                    # Build configuration
├── build_and_verify.sh         # Automated build/verification
└── README_SDCC.md             # This file
```

---

## 🔧 Installation & Setup

### 1. Install SDCC

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install sdcc
```

**macOS (Homebrew):**
```bash
brew install sdcc
```

**Windows:**
- Download from: http://sdcc.sourceforge.net/
- Add SDCC bin directory to PATH

**Verify installation:**
```bash
sdcc --version
# Expected: SDCC 4.x.x or higher
```

### 2. Install Build Tools

```bash
sudo apt-get install make build-essential
```

### 3. Install Flash Programmer (Optional)

For flashing to hardware:
```bash
pip install stcgal  # For STC-compatible MS51 programming
```

---

## 🏗️ Building the Project

### Quick Build

```bash
cd sdcc/
make
```

### Build with Verification

```bash
./build_and_verify.sh
```

This script performs:
- Pre-build checks (tools, files)
- Compilation
- Post-build verification
- Code analysis
- Memory usage report

### Build Targets

```bash
make            # Build project (default)
make clean      # Remove all build artifacts
make size       # Display memory usage
make help       # Show all available targets
make debug      # Build with debugging symbols
make production # Build with maximum optimization
make check      # Syntax check only (no linking)
```

---

## 📊 Expected Build Output

### Successful Build

```
=== Compiling SDCC Project: voltmeter_sdcc ===
...
=== Compilation Complete ===
=== Converting to Intel HEX format ===
=== HEX file created: build/voltmeter_sdcc.hex ===

Memory Usage:
CODE:   ~8000-12000 bytes (of 16384 available)
DATA:   ~100-150 bytes (of 256 available)
```

### Warning Signs

**⚠️ If you see:**
- `stack overflow` → Reduce local variables
- `code size exceeded` → Enable --opt-code-size
- `undefined symbol` → Missing SFR definition
- Warnings about pointer types → Review casts

---

## 🔍 Verifying the Build

### 1. Check Output Files

```bash
ls -lh build/
```

Expected files:
- `voltmeter_sdcc.hex` (ready for flashing)
- `voltmeter_sdcc.ihx` (intermediate format)
- `voltmeter_sdcc.map` (memory layout)
- `voltmeter_sdcc.mem` (memory usage)

### 2. Review Memory Usage

```bash
cat build/voltmeter_sdcc.mem
```

Check that:
- CODE < 16384 bytes (16KB limit)
- IDATA < 256 bytes (internal RAM limit)

### 3. Check for Warnings

```bash
grep -i warning build.log
```

Ideally: **ZERO warnings**

---

## 🚀 Flashing to Hardware

### ⚠️ PRE-FLASH CHECKLIST

- [ ] Backup existing firmware
- [ ] Build completed with ZERO errors
- [ ] Code protection is DISABLED (default)
- [ ] Watchdog is DISABLED (default)
- [ ] Have recovery plan if chip becomes unresponsive

### Using stcgal (Recommended)

```bash
# Connect programmer to MS51
# Identify port: ls /dev/ttyUSB*

stcgal -p /dev/ttyUSB0 -P stc15 build/voltmeter_sdcc.hex
```

### Using Nuvoton ICP Programmer

1. Open Nuvoton ICP software
2. Select MS51FB9AE
3. Load `build/voltmeter_sdcc.hex`
4. Click "Program"

### First Flash Protocol

**DO NOT** skip these steps:

1. Flash firmware
2. **DO NOT** power cycle yet
3. Verify flash successful (no errors)
4. **NOW** power cycle
5. Immediately observe:
   - Does display light up?
   - Any segments visible?
   - Any errors?

If **NOTHING** happens:
- Reflash backup immediately
- Check clock settings
- Verify IO configuration

---

## 🧪 Testing Procedure

### Test 1: Display Functionality
- [ ] All 3 digits light up
- [ ] No ghosting between digits
- [ ] Brightness is acceptable
- [ ] No flicker

### Test 2: Voltage Reading
- [ ] "O-P" label displays
- [ ] Apply 0V → should read ~0
- [ ] Apply 3.3V → should read ~3.3
- [ ] Apply 5.0V → should read ~5.0
- [ ] Readings are stable

### Test 3: Channel Switching
- [ ] Auto-switches from O/P to I/P
- [ ] Both channels read correctly
- [ ] Cycle repeats continuously

### Test 4: Button Calibration
- [ ] INC button increments value
- [ ] DEC button decrements value
- [ ] Timeout saves calibration

### Test 5: EEPROM Persistence
- [ ] Set calibration offset
- [ ] Power cycle
- [ ] Calibration persists

---

## 🐛 Troubleshooting

### Compilation Errors

**Error:** `undefined reference to...`
- **Cause:** Missing function or SFR
- **Fix:** Check header files included

**Error:** `stack overflow in function`
- **Cause:** Too many local variables
- **Fix:** Use static variables or reduce locals

**Error:** `code size exceeded`
- **Cause:** Program too large
- **Fix:** Enable --opt-code-size in Makefile

### Hardware Issues

**Problem:** No display after flash
- Check IO_Init() executed
- Verify Timer0_Delay timing
- Test with oscilloscope on P0

**Problem:** Garbled display
- Adjust Timer0_Delay timing
- Check digit multiplexing speed
- Verify segment patterns

**Problem:** Wrong voltage readings
- Check ADC initialization
- Verify VDD calculation
- Test with known voltage source

**Problem:** Buttons don't work
- Verify P2.0 RESET disabled (AUXR0)
- Check button pin modes
- Test with multimeter

---

## 📖 Key Differences from Keil Version

### Syntax Changes

| Keil C51 | SDCC |
|----------|------|
| `sbit name = address;` | `__sbit __at (address) name;` |
| `sfr name = address;` | `__sfr __at (address) name;` |
| `bit` | `__bit` |
| `set_XXX_YYY` macro | Direct bit assignment |
| `#pragma asm` | `__asm ... __endasm` |

### Behavioral Differences

1. **Memory Model:** SDCC defaults to --model-small
2. **Optimization:** SDCC optimizes differently than Keil
3. **Stack Usage:** SDCC may use more stack space
4. **Code Size:** SDCC code typically 10-20% larger

### Files Replaced

- `SFR_Macro_MS51_16K_keil.h` → `ms51_sdcc.h` + `sfr_macro_sdcc.h`
- `Function_define_MS51_16K_keil.h` → `function_define_sdcc.h`
- `delay.c` (Keil) → `delay_sdcc.c` (SDCC)
- `voltmeter_refactored.c` → `voltmeter_sdcc.c`

---

## 🔒 Production Build

**WARNING:** Only enable after ALL testing passes!

### Enable Code Protection

1. Edit `src/voltmeter_sdcc.c`
2. Change:
   ```c
   // #define BUILD_PRODUCTION
   ```
   to:
   ```c
   #define BUILD_PRODUCTION
   ```

3. Rebuild: `make clean && make`
4. **Test on spare chip first!**
5. Flash production firmware
6. **Chip will be permanently locked!**

### Enable All Features

Production build enables:
- Code protection (locks chip)
- EEPROM calibration storage
- Watchdog timer
- 220V voltage multiplier (if hardware supports)
- Enhanced filtering

---

## 📚 Additional Resources

### Documentation

- `../SDCC_MIGRATION_CHECKLIST.md` - Comprehensive migration guide
- Makefile comments - Build options explained
- Source code comments - Function documentation

### External Links

- [SDCC Manual](http://sdcc.sourceforge.net/doc/sdccman.pdf)
- [MS51 Datasheet](https://www.nuvoton.com/products/microcontrollers/8bit-8051-mcus/)
- [stcgal Documentation](https://github.com/grigorig/stcgal)

### Support

- SDCC Mailing List: sdcc-user@lists.sourceforge.net
- Project Issues: [Create GitHub issue]

---

## ✅ Final Checklist Before Production

- [ ] Code compiles with ZERO errors
- [ ] Code compiles with ZERO warnings
- [ ] All hardware tests pass
- [ ] Calibration works and persists
- [ ] 24-hour burn-in test passed
- [ ] Tested across voltage range (0-999)
- [ ] Both channels tested
- [ ] Backup firmware available
- [ ] Spare chips available (in case of brick)
- [ ] Code protection decision made

---

## 📄 License

Same license as original Voltmeter project.

---

## 🙏 Acknowledgments

- Original Keil code by [Original Author]
- SDCC port: 2025-11-19
- Nuvoton for MS51 support files
- SDCC development team

---

**Good luck with your SDCC build! 🚀**

*Remember: When in doubt, consult the checklist!*
