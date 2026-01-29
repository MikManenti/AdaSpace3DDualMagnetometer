# Dual Magnetometer Implementation Summary

## Changes Made

This implementation adds support for dual magnetometer configuration to improve SpaceMouse precision and 6DOF motion separation.

### Code Changes

#### 1. AdaSpace3D.ino

**New Data Structures:**
- `KalmanFilter` struct: Implements 1D Kalman filter for noise reduction
- Separate `MagCalibration` instances for each sensor: `magCalSolder`, `magCalCable`
- Separate `SensorWatchdog` instances: `watchdogSolder`, `watchdogCable`
- Six Kalman filter instances for 6DOF: `kalman_tx`, `kalman_ty`, `kalman_tz`, `kalman_rx`, `kalman_ry`, `kalman_rz`
- Boolean flags: `bothSensorsActive`, `cableSensorActive`

**Modified Functions:**
- `setup()`: Initialize both sensors, detect configuration, show status via LED colors
- `calibrateMagnetometer()`: Calibrate both sensors simultaneously
- `resetMagnetometer(bool isCable)`: Accept parameter to specify which sensor to reset
- `readAndSendMagnetometerData()`: Main dual sensor fusion logic

**New Functions:**
- `readAndSendSingleMagnetometer()`: Fallback for single sensor mode (legacy behavior)

**Key Features Implemented:**
1. **Automatic sensor detection**: Detects 0, 1, or 2 sensors on startup
2. **Coordinate transformation**: Applies 90° rotation correction for cable sensor
3. **Sensor fusion**: Averages for translation, differences for rotation
4. **Kalman filtering**: Reduces noise on all 6 axes
5. **Predominant movement detection**: Uses argmax to send only strongest movement
6. **Backward compatibility**: Falls back to single sensor mode automatically

#### 2. UserConfig.h

**New Configuration Parameters:**
- `CONFIG_ROT_DEADZONE`: Rotation deadzone threshold (1.5)
- `CONFIG_MIN_MOVEMENT_THRESHOLD`: Minimum movement to trigger HID output (0.5)

**Enhanced Documentation:**
- Added detailed comments for all sensitivity and deadzone parameters
- Explained adjustment recommendations

#### 3. README.md

**Updates:**
- Added dual magnetometer feature to feature list
- Added LED status indicators section
- Added tips for dual vs single sensor modes
- Referenced new DUAL-MAGNETOMETER.md guide

#### 4. DUAL-MAGNETOMETER.md (New File)

Comprehensive guide covering:
- Hardware setup and sensor placement
- How sensor fusion works
- Configuration parameter explanations
- Troubleshooting guide
- Advanced tuning options

## Technical Details

### Coordinate Transformation

Cable sensor is rotated 90° clockwise:
```
X' = Y
Y' = -X
Z' = Z
```

### Sensor Fusion Algorithms

**Translation (average):**
```
Tx = (X_solder + X'_cable) / 2
Ty = (Y_solder + Y'_cable) / 2
Tz = (Z_solder + Z'_cable) / 2
```

**Rotation (context-aware) - Version 4:**
```
// Evaluate which sensor varies MORE (not just difference)
z_solder_abs = |Z_solder|
z_cable_abs = |Z'_cable|

if z_cable_abs > z_solder_abs × RATIO:
    Rx = Z'_cable    // Cable varies more → Pitch
    Ry = 0
elif z_solder_abs > z_cable_abs × RATIO:
    Rx = 0
    Ry = Z_solder    // Solder varies more → Roll
else:
    Rx = (Z_solder - Z'_cable) × 0.5    // Ambiguous case
    Ry = (Z'_cable - Z_solder) × 0.5

Rz = (X_solder - Y_solder) - (X'_cable - Y'_cable)  // Yaw (unchanged)
```

**Note**: 
- Version 4 redesigns rotation logic based on user insight: "dovremmo valutare quale sensore ha la variazione più alta"
- Now looks at which individual sensor varies MORE, not just the difference between them
- Eliminates ambiguity by assigning movement to the axis whose sensor shows dominant change
- Configurable ratio threshold (default 1.3 = 30% more variation required)
- Version 3 fixed Rx/Ry swap
- Version 2 fixed cross-talk where Ry was detected as Tz and Rx was swapped with Ty

### Kalman Filter

Implements standard 1D Kalman filter:
- Process noise covariance (q): 0.01
- Measurement noise covariance (r): 0.1
- Updates each measurement with prediction and correction steps

### Predominant Movement Detection

1. Apply deadzones to all 6 DOF
2. Calculate absolute values: `|Tx|, |Ty|, |Tz|, |Rx|, |Ry|, |Rz|`
3. Find maximum using argmax
4. Send only that movement if above threshold
5. Zero out all other axes

## Testing Recommendations

### Unit Tests (if hardware available)
1. Test single sensor detection (cable only, solder only)
2. Test dual sensor detection
3. Verify LED colors on startup
4. Test coordinate transformation correctness
5. Verify Kalman filter reduces noise
6. Test predominant movement selection

### Integration Tests
1. Verify HID reports are sent correctly
2. Test in actual 3D software (Fusion360, Blender, etc.)
3. Verify smooth movement without jitter
4. Test all 6 DOF separately
5. Verify no cross-talk between axes

### Calibration Tests
1. Power cycle multiple times, verify consistent calibration
2. Test calibration with different rest positions
3. Verify calibration failure handling

## Version History

### Version 5 - Asymmetric Scaling (2026-01-28)

**Problem**: Movements where magnets move away from sensors (Rx forward, Ry left, Tz down) were slower than opposite movements due to non-linear magnetic field strength vs. distance.

**Solution**: Added directional scaling multipliers that apply different sensitivity based on movement sign:

```cpp
// UserConfig.h - New parameters
#define CONFIG_RX_POSITIVE_MULT  1.5   // Boost for Rx forward
#define CONFIG_RX_NEGATIVE_MULT  1.0   // Normal for Rx backward
#define CONFIG_RY_POSITIVE_MULT  1.5   // Boost for Ry left
#define CONFIG_RY_NEGATIVE_MULT  1.0   // Normal for Ry right
#define CONFIG_TZ_POSITIVE_MULT  1.0   // Normal for Tz up
#define CONFIG_TZ_NEGATIVE_MULT  1.5   // Boost for Tz down
```

**Implementation**: Modified scaling logic to check sign of movement before applying scale factor. Default 1.5x boost for "away" directions compensates for weaker signal at greater distances.

### Version 4 - Context-Aware Rotation

Redesigned rotation detection to evaluate which sensor varies MORE, eliminating Rx/Ry ambiguity.

### Version 3 - Axis Assignment Fix

Corrected Rx/Ry axis assignments after user testing revealed they were swapped.

### Version 2 - Z-Axis Differential

Changed rotation detection from XY-plane to Z-axis differences, eliminating translation/rotation cross-talk.

### Version 1 - Initial Implementation

Basic dual magnetometer support with sensor fusion and Kalman filtering.

## Backward Compatibility

The firmware maintains full backward compatibility:
- Single sensor mode works identically to original firmware
- Configuration parameters have sensible defaults
- No breaking changes to existing hardware setups
- Asymmetric multipliers default to 1.5x/1.0x which can be adjusted or disabled (set all to 1.0)

## Future Enhancements (Optional)

1. Adjustable Kalman filter parameters via UserConfig.h
2. Multiple predominant movement modes (send top 2, weighted combination, etc.)
3. Configurable sensor positions (not just 3 o'clock / 6 o'clock)
4. Serial debugging output for sensor values
5. Dynamic deadzone adjustment based on movement history
6. Temperature compensation if sensors support it

## Known Limitations

1. Network issues prevented Arduino CLI compilation test in CI environment
2. Requires physical hardware for full testing
3. Rotation calculations assume specific sensor positions (3 o'clock and 6 o'clock)
4. Yaw detection may need tuning based on physical setup

## Files Changed

- AdaSpace3D.ino (269 lines added, 51 removed)
- UserConfig.h (9 lines added, 3 removed)
- README.md (updated with dual sensor features)
- DUAL-MAGNETOMETER.md (new comprehensive guide)
- IMPLEMENTATION-SUMMARY.md (this file)

## Conclusion

This implementation successfully adds dual magnetometer support with sensor fusion, Kalman filtering, and predominant movement detection while maintaining backward compatibility with single sensor configurations.
