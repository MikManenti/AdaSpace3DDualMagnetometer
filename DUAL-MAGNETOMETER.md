# Dual Magnetometer Configuration Guide

## Overview

This firmware supports an advanced dual magnetometer configuration that significantly improves the SpaceMouse's ability to detect and separate movements across all 6 degrees of freedom (DOF):
- **Translation**: Tx (left/right), Ty (forward/back), Tz (up/down)
- **Rotation**: Rx (pitch), Ry (roll), Rz (yaw)

## Hardware Setup

### Sensor Placement

The system uses two TLx493D magnetometers positioned under the knob:

1. **Sensor 1 (Soldered)**: Located at the 3 o'clock position
   - Connected via soldered headers to the main Wire (I2C) bus
   - Standard orientation (no transformation needed)

2. **Sensor 2 (Cabled)**: Located at the 6 o'clock position
   - Connected via Stemma QT cable to Wire1 (I2C1) bus
   - **Important**: Physically rotated 90° clockwise
   - Firmware automatically transforms coordinates

### Magnet Positioning

Each magnetometer requires a magnet positioned approximately 7-8mm above it (vertical distance at rest). The magnets should be:
- Same type and strength for both positions
- Properly aligned with sensor centers
- Securely attached to the moving knob

## How It Works

### 1. Automatic Calibration

On startup, the firmware:
- Detects which sensors are connected (one or both)
- Displays status via LED color:
  - **Blue**: Both sensors active (dual mode)
  - **Green**: Cable sensor only (single mode)
  - **Cyan**: Solder sensor only (single mode)
- Calibrates both sensors simultaneously
- Calculates neutral position for each sensor

### 2. Coordinate Transformation

The cable sensor is rotated 90° clockwise, so the firmware transforms its readings:

```
X' = Y
Y' = -X
Z' = Z (unchanged)
```

This ensures both sensors report data in the same coordinate frame.

### 3. Sensor Fusion

The firmware combines data from both sensors using different strategies:

**Translation (Tx, Ty, Tz)**:
- Calculated as the average of both sensors
- Represents the overall movement of the knob
- Formula: `T = (Sensor1 + Sensor2_transformed) / 2`

**Rotation (Rx, Ry, Rz)**:
- Calculated from differential measurements between sensors
- Leverages the spatial separation (90° apart) to detect rotation
- When the knob tilts, the magnet moves closer to one sensor and farther from the other
- This creates vertical (Z-axis) magnetic field differences
- Revised formulas (v3 - swapped Rx/Ry):
  - `Rx = Z_solder - Z_cable` (pitch - forward/back tilt)
  - `Ry = Z_cable - Z_solder` (roll - left/right tilt)
  - `Rz = (X_solder - Y_solder) - (X_cable - Y_cable)` (yaw - twist)

### 4. Kalman Filtering

Each of the 6 DOF has its own Kalman filter that:
- Reduces sensor noise
- Smooths jittery readings
- Improves overall precision
- Parameters can be tuned in the code if needed

### 5. Predominant Movement Detection

The firmware identifies which movement is strongest and sends **only that movement** via HID:

```cpp
// Find the axis with maximum absolute value
movements[] = {|Tx|, |Ty|, |Tz|, |Rx|, |Ry|, |Rz|}
predominant = argmax(movements)

// Send only predominant movement
if (predominant == Tx) {
    send_to_HID(tx=value, ty=0, tz=0, rx=0, ry=0, rz=0)
}
```

This eliminates cross-talk and provides clean, separated movements.

## Configuration Parameters

All parameters can be adjusted in `UserConfig.h`:

### Sensitivity Scales
```cpp
#define CONFIG_TRANS_SCALE     100   // Translation sensitivity
#define CONFIG_ZOOM_SCALE      50    // Z-axis (zoom) sensitivity
#define CONFIG_ROT_SCALE       40    // Rotation sensitivity
```
- **Higher values** = More sensitive (larger movements in software)
- **Lower values** = Less sensitive (smaller movements in software)

### Deadzones
```cpp
#define CONFIG_DEADZONE        1.0   // X/Y translation deadzone
#define CONFIG_ZOOM_DEADZONE   2.5   // Z translation deadzone
#define CONFIG_ROT_DEADZONE    1.5   // Rotation deadzone
```
- Filters out unintended micro-movements and sensor noise
- Keep values small since raw sensor values are tiny
- **Higher values** = Less noise but less responsive
- **Lower values** = More responsive but more noise

### Movement Threshold
```cpp
#define CONFIG_MIN_MOVEMENT_THRESHOLD  0.5
```
- Minimum movement magnitude to trigger HID output (dual sensor mode only)
- Prevents sending tiny movements caused by electrical noise
- Adjust based on your physical setup

## Troubleshooting

### Issue: Movements are too sensitive
**Solution**: 
- Lower `CONFIG_TRANS_SCALE`, `CONFIG_ZOOM_SCALE`, or `CONFIG_ROT_SCALE`
- Increase deadzones

### Issue: Not responsive enough
**Solution**:
- Increase scale values
- Lower deadzones
- Lower `CONFIG_MIN_MOVEMENT_THRESHOLD`

### Issue: Jittery or noisy movements
**Solution**:
- Increase deadzones
- Check magnet distances (should be ~7-8mm)
- Ensure magnets are securely attached

### Issue: Wrong movement detected
**Solution**:
- Verify sensor orientations match physical setup
- Check that cable sensor is at 6 o'clock and solder at 3 o'clock
- Ensure magnets are properly aligned with sensor centers

### Issue: Only one sensor detected
**Symptoms**: LED shows green or cyan instead of blue
**Solutions**:
- Check I2C connections
- Verify power supply to sensors
- Ensure Wire and Wire1 buses are properly configured
- Check for I2C address conflicts

## Single Sensor Fallback

If only one sensor is detected, the firmware automatically falls back to legacy single-sensor mode:
- No sensor fusion
- No predominant movement filtering
- Uses simpler mapping (same as original firmware)
- Still benefits from calibration and watchdog features

This ensures the device remains functional even if one sensor fails.

## Advanced Tuning

### Kalman Filter Parameters

For advanced users, Kalman filter parameters can be adjusted in the code:

```cpp
struct KalmanFilter {
  double q = 0.01;     // Process noise covariance (prediction uncertainty)
  double r = 0.1;      // Measurement noise covariance (sensor noise)
  // ...
};
```

- **Increase `q`**: More responsive but noisier
- **Decrease `q`**: Smoother but slower to respond
- **Increase `r`**: More smoothing (trusts sensor less)
- **Decrease `r`**: Less smoothing (trusts sensor more)

### Sensor Fusion Formulas

The rotation calculations can be modified if your sensor placement differs:

```cpp
// These formulas assume:
// - Solder sensor at 3 o'clock (0°)
// - Cable sensor at 6 o'clock (90° clockwise)
// Updated formulas (v3) with Rx/Ry swapped:
double raw_rx = (z_solder - z_cable_transformed);  // Pitch
double raw_ry = (z_cable_transformed - z_solder);  // Roll  
double raw_rz = (x_solder - y_solder) - (x_cable_transformed - y_cable_transformed);  // Yaw
```

Adjust based on your physical configuration.

## Changelog

**Version 3 (2026-01-28)**:
- Fixed Rx/Ry swap issue - roll and pitch axes were inverted
- Swapped the Z-axis difference formulas for Rx and Ry

**Version 2 (2026-01-28)**:
- Fixed cross-talk issue where Ry was detected as Tz and Rx was swapped with Ty
- Revised rotation formulas to use Z-axis differences instead of XY plane differences
- Improved separation between rotational and translational movements

## Benefits of Dual Magnetometer Setup

1. **Improved 6DOF Separation**: Translation and rotation movements are clearly distinguished
2. **Reduced Cross-Talk**: Only predominant movement is sent at a time
3. **Better Precision**: Averaging and differential measurements reduce errors
4. **Noise Reduction**: Kalman filtering on all axes
5. **Yaw Detection**: The differential setup can detect Z-axis rotation (yaw)
6. **Redundancy**: System continues working if one sensor fails

## Support

For issues, questions, or suggestions about the dual magnetometer configuration:
- Join the Discord: http://dsc.gg/axiom3d
- Open a GitHub issue
- Check the main README.md for general information

---

**Note**: This is an advanced configuration. For a simpler setup, the original single-sensor firmware works perfectly fine for most use cases.
