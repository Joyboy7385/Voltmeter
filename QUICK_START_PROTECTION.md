# Quick Start: Code Protection

## 🚀 Fast Track Guide

### For Development (Testing)
```c
// In voltmeter_production.c line 14:
// #define PRODUCTION_BUILD    // ← Keep commented (has //)
```
**Result:** Code NOT protected, can debug and reprogram freely

---

### For Production (Final Products)
```c
// In voltmeter_production.c line 14:
#define PRODUCTION_BUILD    // ← Uncomment (remove //)
```
**Result:** Code PROTECTED after first power-on, locked permanently

---

## 📝 Step-by-Step

### Development Build
1. Open `voltmeter_production.c`
2. Find line ~14: `// #define PRODUCTION_BUILD`
3. **Leave it commented** (with `//`)
4. Build in Keil
5. Flash to chip
6. ✅ Can debug and reprogram anytime

### Production Build
1. Open `voltmeter_production.c`
2. Find line ~14: `// #define PRODUCTION_BUILD`
3. **Remove the `//`** → `#define PRODUCTION_BUILD`
4. Build in Keil (watch for warnings!)
5. Flash to chip
6. **Power on** → Display shows "888" (2s) → "---" (2s)
7. **Power cycle again**
8. 🔒 Code is now LOCKED!

---

## ⚠️ Important

| Action | Development | Production |
|--------|-------------|------------|
| Can read code? | ✅ Yes | ❌ No |
| Can debug? | ✅ Yes | ❌ No |
| Can reprogram? | ✅ Yes | ⚠️ Only after mass erase |
| Display "888"? | ❌ No | ✅ Yes (first time) |

---

## 🧪 Before Production

✅ **MUST DO before enabling protection:**
- [ ] All features tested
- [ ] Run `./test_simulation` (37/37 passing)
- [ ] Voltage readings accurate
- [ ] Calibration works
- [ ] Long-term stability tested (24hr+)
- [ ] Document firmware version

---

## 🔓 Unlocking (Emergency)

**If you need to unlock a protected chip:**

```
⚠️ WARNING: ERASES ALL CODE!

1. Connect ICP programmer
2. Click "Mass Erase"
3. ALL code deleted
4. Can now reprogram
```

---

## 🎯 Quick Check

**Is my chip protected?**

Try reading code with ICP tool:
- Can read → NOT protected
- Error or 0xFF → Protected ✅

---

## Files Reference

| File | Purpose |
|------|---------|
| `voltmeter_production.c` | Main code with protection |
| `CODE_PROTECTION_GUIDE.md` | Full documentation |
| `build.sh` | Automated build script |
| This file | Quick reference |

---

## Common Mistakes

❌ **Forgot to uncomment `PRODUCTION_BUILD`**
- Result: Code not protected
- Fix: Edit line 14, remove `//`

❌ **Enabled protection during development**
- Result: Cannot debug
- Fix: Mass erase, reprogram with debug version

❌ **Didn't test before locking**
- Result: Locked buggy code
- Fix: Mass erase, reprogram with fixed version

---

## Support

**Read full guide:** `CODE_PROTECTION_GUIDE.md`

**Need help?** Check Troubleshooting section in full guide

---

**Last Updated:** 2025-11-17
