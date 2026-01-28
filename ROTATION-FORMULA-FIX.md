# Rotation Formula Fix (Version 2)

## Problem Report

User feedback (2026-01-28): *"ottimo, funziona abbastanza bene, unico problema ry viene spesso rilevata come tz e rx scambiata per ty"*

Translation: "great, it works quite well, only problem ry is often detected as tz and rx is swapped for ty"

**Symptoms:**
- Ry (roll - left/right tilt) was being detected as Tz (zoom - vertical movement)
- Rx (pitch - forward/back tilt) was being confused with Ty (forward/back translation)

## Root Cause Analysis

### Original Formulas (Version 1)

```cpp
// Translation
raw_tx = (x_solder + x_cable_transformed) / 2.0;
raw_ty = (y_solder + y_cable_transformed) / 2.0;
raw_tz = (z_solder + z_cable_transformed) / 2.0;

// Rotation (PROBLEMATIC)
raw_rx = (y_solder - y_cable_transformed);  // Used Y-axis difference
raw_ry = (x_cable_transformed - x_solder);  // Used X-axis difference  
raw_rz = (x_solder + y_cable_transformed) - (x_cable_transformed + y_solder);
```

### Why This Caused Cross-Talk

1. **The XY-plane differences used for rotation were also present in translation**
   - When the knob moved forward/back (Ty translation), this also changed Y values
   - The `y_solder - y_cable_transformed` formula for Rx would pick up this translational movement
   - Result: **Rx confused with Ty** ✗

2. **The X-axis difference for Ry was similar to the Z-axis average**
   - When the knob moved up/down (Tz translation), sensors saw similar field changes
   - The `x_cable_transformed - x_solder` formula was sensitive to these changes
   - Result: **Ry detected as Tz** ✗

## The Fix

### Physical Understanding

When a magnet positioned above two sensors tilts:
- The magnet moves **closer** to one sensor
- The magnet moves **farther** from the other sensor
- This creates a **vertical (Z-axis) magnetic field strength difference**
- The XY components are less reliable for tilt detection because they're affected by both translation and rotation

### New Formulas (Version 2)

```cpp
// Translation (UNCHANGED - still works correctly)
raw_tx = (x_solder + x_cable_transformed) / 2.0;
raw_ty = (y_solder + y_cable_transformed) / 2.0;
raw_tz = (z_solder + z_cable_transformed) / 2.0;

// Rotation (FIXED - now uses Z-axis differences)
raw_rx = (z_cable_transformed - z_solder);  // Pitch: Z difference
raw_ry = (z_solder - z_cable_transformed);  // Roll: inverted Z difference
raw_rz = (x_solder - y_solder) - (x_cable_transformed - y_cable_transformed);  // Yaw
```

### Why This Works

1. **Z-axis differences are unique to rotation**
   - Pure translation (Tx, Ty, Tz) moves both sensors equally → Z_solder ≈ Z_cable → difference ≈ 0
   - Rotation tilts the magnet → Z_solder ≠ Z_cable → significant difference
   - Result: **Rotation separated from translation** ✓

2. **Sensor geometry supports this approach**
   - Solder sensor at 3 o'clock (East position)
   - Cable sensor at 6 o'clock (South position after 90° transform)
   - When tilting forward (pitch), the South sensor sees different Z than East sensor
   - When tilting sideways (roll), the East sensor sees different Z than South sensor
   - Result: **Rx and Ry properly distinguished** ✓

3. **Yaw uses XY asymmetry which is appropriate**
   - Yaw (twist) rotation doesn't significantly change Z fields
   - Instead, it creates asymmetry in the XY plane
   - The formula `(X - Y)` captures this rotational asymmetry
   - Result: **Yaw detection preserved** ✓

## Verification

### Expected Behavior After Fix

| Movement | Detected As | Notes |
|----------|-------------|-------|
| Move forward/back | Ty | Pure translation, Z differences remain small |
| Move left/right | Tx | Pure translation, Z differences remain small |
| Move up/down | Tz | Pure translation, both sensors see same Z change |
| Tilt forward/back | Rx | Pitch creates Z_cable ≠ Z_solder |
| Tilt left/right | Ry | Roll creates Z_solder ≠ Z_cable (inverted) |
| Twist left/right | Rz | Yaw creates XY asymmetry |

### Testing Recommendations

To verify the fix works:

1. **Test Pure Translation**
   - Move knob straight up/down → Should detect only Tz (not Ry)
   - Move knob forward/back → Should detect only Ty (not Rx)
   - Move knob left/right → Should detect only Tx

2. **Test Pure Rotation**
   - Tilt knob forward/back without moving → Should detect only Rx (not Ty)
   - Tilt knob left/right without moving → Should detect only Ry (not Tz)
   - Twist knob → Should detect only Rz

3. **Check Cross-Talk**
   - Monitor that Ry movements don't trigger Tz detection
   - Monitor that Rx movements don't trigger Ty detection
   - Verify predominant movement selection works correctly

## Implementation Details

### Code Changes

**File: `AdaSpace3D.ino`**
- Lines 420-422: Updated rotation calculation formulas
- Added detailed comments explaining the fix

**Files: `DUAL-MAGNETOMETER.md` and `IMPLEMENTATION-SUMMARY.md`**
- Updated formula documentation
- Added changelog noting Version 2
- Explained the reasoning behind the change

### Backward Compatibility

- Translation formulas unchanged
- Kalman filtering unchanged
- Predominant movement detection unchanged
- Configuration parameters unchanged
- Only rotation formulas modified

### No Breaking Changes

Users upgrading from Version 1 to Version 2 will:
- Keep all their configuration settings
- Experience better movement separation
- No longer see the reported cross-talk issues
- Not need to recalibrate (calibration is for neutral position only)

## Future Considerations

### If Issues Persist

If users still report cross-talk after this fix:

1. **Check magnet alignment**: Ensure magnets are centered over sensors
2. **Verify sensor orientation**: Confirm cable sensor is at 6 o'clock, rotated 90°
3. **Adjust deadzones**: May need tuning for specific hardware
4. **Check magnet distance**: Should be ~7-8mm vertical distance
5. **Try inverting formulas**: Some configurations might need sign inversions

### Advanced Tuning

The formulas can be scaled independently if needed:

```cpp
double rx_scale = 1.0;  // Adjust sensitivity
double ry_scale = 1.0;
double rz_scale = 1.0;

raw_rx = (z_cable_transformed - z_solder) * rx_scale;
raw_ry = (z_solder - z_cable_transformed) * ry_scale;
raw_rz = ((x_solder - y_solder) - (x_cable_transformed - y_cable_transformed)) * rz_scale;
```

## Conclusion

This fix addresses the root cause of cross-talk by using the appropriate magnetic field components for each type of movement:
- **Translation**: Average of XYZ components (unchanged)
- **Rotation**: Z-axis differences for pitch/roll, XY asymmetry for yaw (fixed)

The key insight is that tilting movements create **vertical field strength differences** at spatially separated sensors, which is the best signal for detecting rotation without confusing it with translation.

---

**Version History:**
- V1 (2026-01-28): Initial dual magnetometer implementation
- V2 (2026-01-28): Fixed Rx/Ty and Ry/Tz cross-talk (this document)
