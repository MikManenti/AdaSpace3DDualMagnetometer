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
- Calculated using context-aware differential measurements (v4)
- Leverages the spatial separation (90° apart) to detect rotation
- When the knob tilts, the magnet moves closer to one sensor and farther from the other
- This creates vertical (Z-axis) magnetic field differences
- **New approach (v4)**: Evaluates which sensor varies MORE
  - If cable sensor (bottom at 6 o'clock) varies significantly more → **Pitch (Rx)**
  - If solder sensor (right at 3 o'clock) varies significantly more → **Roll (Ry)**
  - Uses configurable ratio threshold (default 1.3 = 30% more variation required)
  - This prevents ambiguity and cross-talk between rotation axes
  - `Rz = (X_solder - Y_solder) - (X_cable - Y_cable)` (yaw - twist, unchanged)

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

### Asymmetric Scaling (New in v5)
```cpp
#define CONFIG_RX_POSITIVE_MULT  1.5   // Rx forward (magnet away)
#define CONFIG_RX_NEGATIVE_MULT  1.0   // Rx backward (magnet closer)
#define CONFIG_RY_POSITIVE_MULT  1.5   // Ry left (magnet away)
#define CONFIG_RY_NEGATIVE_MULT  1.0   // Ry right (magnet closer)
#define CONFIG_TZ_POSITIVE_MULT  1.0   // Tz up (magnet closer)
#define CONFIG_TZ_NEGATIVE_MULT  1.5   // Tz down (magnet away)
```
- Compensates for non-linear magnetic field strength vs. distance
- When magnets move **away** from sensors, signal is weaker → use higher multiplier
- When magnets move **closer** to sensors, signal is stronger → use lower multiplier
- **Default 1.5x boost** for "away" movements, 1.0x for "closer" movements
- Set both to 1.0 for symmetric scaling (disable feature)
- Adjust based on your specific hardware and magnet distances

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

### Rotation Axis Discrimination (New in v4)
```cpp
#define CONFIG_ROTATION_AXIS_RATIO  1.3
```
- Controls how much more one sensor must vary than the other for Rx/Ry determination
- Value of 1.3 means one sensor must vary 30% more than the other
- **Higher values** (e.g., 1.5) = Stricter separation, less cross-talk between Rx/Ry
- **Lower values** (e.g., 1.2) = More sensitive, catches mixed movements but may have cross-talk
- Recommended range: 1.2 to 1.5

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

### Issue: Movements in one direction are slower than the opposite direction
**Solution** (New in v5):
- Adjust asymmetric scaling multipliers
- Increase the multiplier for the slower direction
- Example: If Rx forward is slow, increase `CONFIG_RX_POSITIVE_MULT` (try 1.8 or 2.0)
- If Tz down is slow, increase `CONFIG_TZ_NEGATIVE_MULT`
- Fine-tune values in 0.1 increments until movements feel balanced

### Issue: Rx and Ry are still interfering with each other
**Solution**:
- Increase `CONFIG_ROTATION_AXIS_RATIO` (e.g., to 1.5 or 1.6)
- This makes the system require a stronger difference before assigning to one axis

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
// Version 4 - Context-aware rotation detection
// Evaluates which sensor varies MORE and assigns to appropriate axis
double z_solder_abs = abs(z_solder);
double z_cable_abs = abs(z_cable_transformed);

if (z_cable_abs > z_solder_abs * CONFIG_ROTATION_AXIS_RATIO) {
  // Cable (bottom) varies more → Pitch
  raw_rx = z_cable_transformed;
  raw_ry = 0.0;
} 
else if (z_solder_abs > z_cable_abs * CONFIG_ROTATION_AXIS_RATIO) {
  // Solder (right) varies more → Roll
  raw_rx = 0.0;
  raw_ry = z_solder;
}
else {
  // Ambiguous - use differential with reduced sensitivity
  raw_rx = (z_solder - z_cable_transformed) * 0.5;
  raw_ry = (z_cable_transformed - z_solder) * 0.5;
}
```

Adjust based on your physical configuration.

## Changelog

**Version 5 (2026-01-28)**:
- Added asymmetric scaling multipliers for directional movements
- Compensates for non-linear magnetic field (weaker when magnet is farther)
- Six new configuration parameters: `CONFIG_RX_POSITIVE_MULT`, `CONFIG_RX_NEGATIVE_MULT`, etc.
- Default 1.5x boost for "away" movements (Rx forward, Ry left, Tz down)
- Separately tunable for each axis and direction
- Fixes issue where movements away from sensors were slower

**Version 4 (2026-01-28)**:
- Completely redesigned rotation detection logic based on user feedback
- Now evaluates which sensor shows MORE variation (not just difference)
- Assigns movement to Rx or Ry based on which sensor varies significantly more
- Added `CONFIG_ROTATION_AXIS_RATIO` parameter for tuning (default 1.3)
- Eliminates ambiguity between pitch and roll movements
- Fallback to differential method when both sensors vary similarly

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
