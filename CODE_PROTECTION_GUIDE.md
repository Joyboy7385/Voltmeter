# Code Protection Guide for MS51 Voltmeter

**Version:** 1.0
**Date:** 2025-11-17
**Target MCU:** MS51FB9AE

---

## Overview

This guide explains how to protect your voltmeter firmware from code extraction and reverse engineering using the MS51's built-in security features.

---

## Security Features Implemented

### 1. **Flash Memory Lock (LOCK Bit)**
- Prevents reading flash via ISP/ICP programmers
- Blocks code dumping attempts
- Requires mass erase to unlock (destroys all code)

### 2. **On-Chip Debug Disable (OCDEN)**
- Disables JTAG/OnCD debugging interface
- Prevents runtime code inspection
- Blocks breakpoint debugging

### 3. **Automatic Protection Enablement**
- Production firmware automatically locks itself on first run
- Visual feedback via display ("888" → "---")
- No manual intervention needed

---

## Build Configurations

### Development Build (Default)
```c
// In voltmeter_production.c
// Leave PRODUCTION_BUILD commented out
// #define PRODUCTION_BUILD

Features:
✓ Code protection: DISABLED
✓ Debugging: ENABLED
✓ ISP/ICP: Full access
✓ Code can be read back
```

**Use for:**
- Software development
- Testing and debugging
- Code verification
- Frequent reprogramming

### Production Build
```c
// In voltmeter_production.c
// Uncomment PRODUCTION_BUILD
#define PRODUCTION_BUILD

Features:
✓ Code protection: ENABLED
✗ Debugging: DISABLED
✗ ISP/ICP: Cannot read code
✗ Code locked after first power-on
```

**Use for:**
- Final products
- Customer shipments
- Commercial deployment
- IP protection

---

## How It Works

### First Power-On Sequence

When production firmware runs for the first time:

1. **Check Protection Status**
   ```
   System checks if code is already locked
   ```

2. **Visual Indicator**
   ```
   Display shows: "888" (2 seconds)
   Indicates: Protection being enabled
   ```

3. **Enable Protection**
   ```
   Writes CONFIG0 register
   Sets LOCK=0 and OCDEN=0
   ```

4. **Confirmation**
   ```
   Display shows: "---" (2 seconds)
   Indicates: Protection enabled
   ```

5. **Normal Operation**
   ```
   Device runs normally
   Protection active after next power cycle
   ```

### Subsequent Power-Ons

- Protection already enabled
- No "888" or "---" display
- Device starts normally
- Code remains locked

---

## Programming Instructions

### Method 1: Using Keil µVision (Recommended)

#### Development Build:
```
1. Open voltmeter_production.c
2. Ensure PRODUCTION_BUILD is commented out
3. Build → Rebuild All
4. Flash → Download
5. Test and debug normally
```

#### Production Build:
```
1. Open voltmeter_production.c
2. Uncomment: #define PRODUCTION_BUILD
3. Build → Rebuild All
4. Verify warnings:
   "PRODUCTION BUILD: Code protection will be enabled!"
   "Make sure all testing is complete before flashing!"
5. Flash → Download
6. Power cycle device
7. Observe "888" → "---" on display
8. Power cycle again
9. Code is now protected!
```

### Method 2: Using Nuvoton ICP Tool

#### Step-by-Step:

1. **Prepare HEX File**
   ```bash
   # Build production version first
   keil build voltmeter_production.uvproj
   ```

2. **Open ICP Tool**
   ```
   - Launch Nuvoton ICP Programming Tool
   - Select: MS51FB9AE
   ```

3. **Load Firmware**
   ```
   File → Open → Select voltmeter_production.hex
   ```

4. **Configure Protection** (Optional - Code does this automatically)
   ```
   Config Tab:
   ☑ LOCK (bit 7 = 0)
   ☑ OCDEN Disable (bit 6 = 0)
   ```

5. **Program Device**
   ```
   - Click "Program"
   - Wait for completion
   - Verify successful
   ```

6. **Test Protection**
   ```
   - Click "Read" or "Verify"
   - Should get error or 0xFF
   - If successful, protection NOT enabled
   ```

---

## Verification Testing

### Test 1: Visual Confirmation

**Expected Behavior:**
```
Power On → "888" (2s) → "---" (2s) → Normal operation
```

**If no "888"/"---":**
- PRODUCTION_BUILD not defined
- Or protection already enabled

### Test 2: Read-Back Test

**Using ICP Tool:**
```bash
1. Connect programmer to locked device
2. Try to read flash
3. Expected: Read error or all 0xFF
4. If you can read code → NOT PROTECTED!
```

### Test 3: Debug Test

**Using Keil Debugger:**
```
1. Try to start debug session
2. Expected: Connection error
3. If debugger connects → NOT PROTECTED!
```

---

## Production Checklist

Before enabling protection, verify:

- [ ] All features tested and working
- [ ] Voltage readings accurate (±1V)
- [ ] All 37 automated tests passing
- [ ] Calibration functionality verified
- [ ] Display working correctly
- [ ] Buttons responding properly
- [ ] VDD compensation tested (4.5-5.5V)
- [ ] Long-term stability tested (>24 hours)
- [ ] Temperature range tested (if applicable)
- [ ] Final firmware version documented

After enabling protection:

- [ ] "888" and "---" displayed on first power-on
- [ ] Read-back test fails (cannot read code)
- [ ] Debug connection fails
- [ ] Device operates normally
- [ ] Voltage measurements still accurate

---

## Unlocking for Repairs/Updates

### ⚠️ WARNING: Unlocking ERASES ALL CODE!

To unlock a protected chip:

### Method 1: Mass Erase via ICP Tool

```
1. Connect ICP programmer
2. Select "Mass Erase" option
3. Click "Erase"
4. All flash memory erased (code lost!)
5. CONFIG bits reset to default (unlocked)
6. Can now reprogram with new firmware
```

### Method 2: ISP Command Sequence

```
1. Enter ISP mode (specific to your programmer)
2. Send mass erase command
3. Wait for erase completion
4. Flash is now blank and unlocked
```

---

## Troubleshooting

### Problem: Cannot Program Protected Chip

**Symptoms:**
- Programming fails
- Verification errors
- "Device locked" message

**Solution:**
1. Perform mass erase first
2. Then program new firmware

### Problem: Code Not Protected After Programming

**Check:**
- [ ] PRODUCTION_BUILD defined?
- [ ] Compiler warnings shown?
- [ ] "888"/"---" displayed on power-on?
- [ ] Power cycled after first run?

**Test:**
- Try reading code back
- If successful, protection not enabled

### Problem: Protection Enabled During Development

**If you accidentally locked development chip:**

**Solution:**
1. Mass erase using ICP tool
2. Reprogram with development build
3. Continue testing

**Prevention:**
- Keep separate development and production chips
- Label chips clearly
- Use version control for builds

---

## Security Best Practices

### 1. **Multi-Layer Protection**

```
Layer 1: Flash LOCK bit ✓
Layer 2: OCDEN disable ✓
Layer 3: Physical protection (epoxy blob)
Layer 4: Remove ISP header on production PCB
Layer 5: Tamper detection (optional)
```

### 2. **Development vs Production**

**Development Chips:**
- Never enable protection
- Keep for testing and debugging
- Clearly labeled

**Production Chips:**
- Always enable protection
- One-way process
- Document which units are protected

### 3. **Version Tracking**

```c
/* Add to firmware */
#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  0
#define FW_BUILD_DATE     "2025-11-17"
#define FW_BUILD_NUMBER   1001

/* Store in unprotected area for identification */
code unsigned char __at (0x7F00) fw_info[] = {
    FW_VERSION_MAJOR,
    FW_VERSION_MINOR,
    (FW_BUILD_NUMBER >> 8),
    (FW_BUILD_NUMBER & 0xFF)
};
```

### 4. **Factory Programming Process**

```
Step 1: Build production firmware
Step 2: Run all automated tests
Step 3: Program test unit
Step 4: Verify protection works
Step 5: Program production batch
Step 6: Functional test each unit
Step 7: Power cycle to activate protection
Step 8: Verify cannot read code
Step 9: Package and ship
```

---

## CONFIG Register Details

### CONFIG0 Register (Address 0x0000)

```
Bit 7: LOCK
  0 = Flash locked (protected)
  1 = Flash unlocked (can read)

Bit 6: OCDEN (On-Chip Debug Enable)
  0 = Debug disabled
  1 = Debug enabled

Bits 5-0: Other config bits
```

### Production Setting:
```c
CONFIG0 = 0x7E;
// Binary: 0111 1110
// LOCK=0 (locked)
// OCDEN=0 (disabled)
```

### Development Setting:
```c
CONFIG0 = 0xFF;
// Binary: 1111 1111
// LOCK=1 (unlocked)
// OCDEN=1 (enabled)
```

---

## Advanced: Custom Protection Schemes

### Optional: Encrypted Firmware Updates

For field updates without unlocking:

```c
/* Reserve bootloader area */
#define BOOTLOADER_START  0x0000
#define BOOTLOADER_SIZE   0x0800  // 2KB
#define APP_START         0x0800

/* Bootloader can update application without unlock */
/* Application area not protected, bootloader is protected */
```

### Optional: Authentication

```c
/* Add device-specific secret */
#define DEVICE_SECRET  0x1A2B3C4D  // Unique per device

/* Verify authenticity before critical operations */
unsigned char Authenticate(unsigned long challenge)
{
    return (challenge ^ DEVICE_SECRET) == EXPECTED_RESPONSE;
}
```

---

## Compliance and Legal

### Export Regulations

**Check regulations before shipping internationally:**
- Encryption may require export licenses
- Code protection may have restrictions
- Document security measures for customs

### IP Protection

**Code protection helps protect:**
- Proprietary algorithms
- Calibration methods
- Trade secrets
- Competitive advantage

**But does not protect:**
- External circuit design
- PCB layout (use conformal coating)
- Physical appearance (use patents/trademarks)

---

## Quick Reference

### Enable Protection
```c
#define PRODUCTION_BUILD  // Uncomment this line
```

### Disable Protection
```c
// #define PRODUCTION_BUILD  // Comment out this line
```

### Check if Protected
```c
if (Is_Code_Protected()) {
    // Code is locked
} else {
    // Code is unlocked
}
```

### Force Protection (Advanced)
```c
Enable_Code_Protection();
// Power cycle required for activation
```

---

## Support and Updates

### Getting Help

**If you encounter issues:**
1. Check this guide first
2. Review troubleshooting section
3. Verify your build configuration
4. Test with development build first

### Updating Firmware

**For protected devices:**
1. Mass erase (loses all code)
2. Program new firmware
3. Protection auto-enables on first run

**For development devices:**
1. Just reprogram normally
2. No erase needed

---

## Summary

### ✅ Protection Enabled Successfully When:
- "888" → "---" shown on first power-on
- Cannot read code via ICP
- Cannot connect debugger
- Device operates normally

### ❌ Protection NOT Working If:
- No "888" display on first run
- Can read code back
- Debugger connects successfully
- PRODUCTION_BUILD not defined

### 🔒 Your IP is Protected When:
- Flash LOCK bit = 0
- OCDEN bit = 0
- Read-back test fails
- Debug connection blocked
- (Optional) Physical protection applied

---

**End of Code Protection Guide**

For technical support or questions about code protection,
refer to the MS51FB9AE datasheet section on Flash Security.

**Version:** 1.0
**Last Updated:** 2025-11-17
