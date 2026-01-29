# Asymmetric Scaling Verification (v7)

## User Concern
*"attenzione, hai rimosso lo scaling separato tra positivo e negativo dei seguenti assi rx ry e tz"*

Translation: "attention, you have removed the separate scaling between positive and negative of the following axes rx ry and tz"

## Verification: Asymmetric Scaling is PRESERVED ✅

The asymmetric scaling feature from v5 is **fully preserved** in v7. Here's the proof:

### Configuration Parameters (UserConfig.h)

**Rx Asymmetric Multipliers:**
```cpp
#define CONFIG_RX_POSITIVE_MULT  1.5   // Rx forward (magnet away) - typically needs boost
#define CONFIG_RX_NEGATIVE_MULT  1.0   // Rx backward (magnet closer)
```

**Ry Asymmetric Multipliers:**
```cpp
#define CONFIG_RY_POSITIVE_MULT  1.5   // Ry left (magnet away) - typically needs boost
#define CONFIG_RY_NEGATIVE_MULT  1.0   // Ry right (magnet closer)
```

**Tz Asymmetric Multipliers:**
```cpp
#define CONFIG_TZ_POSITIVE_MULT  1.0   // Tz up (magnet closer)
#define CONFIG_TZ_NEGATIVE_MULT  1.5   // Tz down (magnet away) - typically needs boost
```

### Code Implementation (AdaSpace3D.ino)

**Rx Implementation (lines 509-517):**
```cpp
case 3: // Rx - asymmetric scaling
  if (rx >= 0) {
    // Positive (forward/farther) - use positive multiplier
    out_rx = (int16_t)constrain(rx * CONFIG_RX_SCALE * CONFIG_RX_POSITIVE_MULT, -32767, 32767);
  } else {
    // Negative (backward/closer) - use negative multiplier
    out_rx = (int16_t)constrain(rx * CONFIG_RX_SCALE * CONFIG_RX_NEGATIVE_MULT, -32767, 32767);
  }
  break;
```

**Ry Implementation (lines 518-526):**
```cpp
case 4: // Ry - asymmetric scaling
  if (ry >= 0) {
    // Positive (left/farther) - use positive multiplier
    out_ry = (int16_t)constrain(ry * CONFIG_RY_SCALE * CONFIG_RY_POSITIVE_MULT, -32767, 32767);
  } else {
    // Negative (right/closer) - use negative multiplier
    out_ry = (int16_t)constrain(ry * CONFIG_RY_SCALE * CONFIG_RY_NEGATIVE_MULT, -32767, 32767);
  }
  break;
```

**Tz Implementation (lines 500-508):**
```cpp
case 2: // Tz - asymmetric scaling
  if (tz >= 0) {
    // Positive (up/closer) - use positive multiplier
    out_tz = (int16_t)constrain(tz * CONFIG_TZ_SCALE * CONFIG_TZ_POSITIVE_MULT, -32767, 32767);
  } else {
    // Negative (down/farther) - use negative multiplier
    out_tz = (int16_t)constrain(tz * CONFIG_TZ_SCALE * CONFIG_TZ_NEGATIVE_MULT, -32767, 32767);
  }
  break;
```

## What Changed in v7?

### Before (v5):
- Used **grouped** base scaling: `CONFIG_ROT_SCALE` for all rotations, `CONFIG_ZOOM_SCALE` for Tz
- Applied asymmetric multipliers on top: `CONFIG_RX_POSITIVE_MULT`, etc.

**Example v5 formula:**
```cpp
out_rx = rx * CONFIG_ROT_SCALE * CONFIG_RX_POSITIVE_MULT  // if rx >= 0
```

### After (v7):
- Uses **per-axis** base scaling: `CONFIG_RX_SCALE`, `CONFIG_RY_SCALE`, `CONFIG_TZ_SCALE`, etc.
- **Still** applies asymmetric multipliers on top: `CONFIG_RX_POSITIVE_MULT`, etc.

**Example v7 formula:**
```cpp
out_rx = rx * CONFIG_RX_SCALE * CONFIG_RX_POSITIVE_MULT  // if rx >= 0
```

### Key Point:
The **only change** is:
- `CONFIG_ROT_SCALE` → `CONFIG_RX_SCALE` (for Rx)
- `CONFIG_ROT_SCALE` → `CONFIG_RY_SCALE` (for Ry)
- `CONFIG_ZOOM_SCALE` → `CONFIG_TZ_SCALE` (for Tz)

The **asymmetric multipliers** are **UNCHANGED** and **fully functional**.

## Actual Behavior Examples

### Rx (Pitch)
With defaults: `CONFIG_RX_SCALE = 40`, `CONFIG_RX_POSITIVE_MULT = 1.5`, `CONFIG_RX_NEGATIVE_MULT = 1.0`

- **Forward tilt** (rx = +10): `output = 10 * 40 * 1.5 = 600`
- **Backward tilt** (rx = -10): `output = -10 * 40 * 1.0 = -400`

Result: Forward movement is 1.5x more sensitive than backward ✅

### Ry (Roll)
With defaults: `CONFIG_RY_SCALE = 40`, `CONFIG_RY_POSITIVE_MULT = 1.5`, `CONFIG_RY_NEGATIVE_MULT = 1.0`

- **Left tilt** (ry = +10): `output = 10 * 40 * 1.5 = 600`
- **Right tilt** (ry = -10): `output = -10 * 40 * 1.0 = -400`

Result: Left movement is 1.5x more sensitive than right ✅

### Tz (Zoom)
With defaults: `CONFIG_TZ_SCALE = 50`, `CONFIG_TZ_POSITIVE_MULT = 1.0`, `CONFIG_TZ_NEGATIVE_MULT = 1.5`

- **Up movement** (tz = +10): `output = 10 * 50 * 1.0 = 500`
- **Down movement** (tz = -10): `output = -10 * 50 * 1.5 = -750`

Result: Down movement is 1.5x more sensitive than up ✅

## Conclusion

**The asymmetric scaling is 100% preserved and working correctly.**

Version 7 **added** per-axis base scaling control **on top of** the existing asymmetric multipliers. Users now have:

1. **Per-axis base scaling** (new in v7): `CONFIG_TX_SCALE`, `CONFIG_TY_SCALE`, etc.
2. **Asymmetric directional multipliers** (preserved from v5): `CONFIG_RX_POSITIVE_MULT`, etc.
3. **Per-axis deadzones** (new in v7): `CONFIG_TX_DEADZONE`, etc.

This provides even **MORE control** than v5, not less!

---

**If the user is experiencing different behavior, it may be due to:**
1. Configuration values not set correctly
2. A misunderstanding of how the scaling works
3. A different issue unrelated to the v7 changes

The code review confirms: **Asymmetric scaling is fully functional in v7**.
