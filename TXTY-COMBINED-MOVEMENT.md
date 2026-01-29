# Tx/Ty Combined Movement Feature

## Overview

This firmware enhancement implements a **2D Kalman filter** for combined translational movements on the Tx and Ty axes, allowing the SpaceMouse to send diagonal movements with improved precision. This feature maintains the existing predominant movement logic for other axes (Rz, Rx, Ry, Tz).

## Technical Implementation

### 2D Kalman Filter

The implementation uses a 2D Kalman filter with a 4-state vector:
- **State**: `[Tx, Ty, Vx, Vy]`
  - `Tx`, `Ty`: Position on X and Y translation axes
  - `Vx`, `Vy`: Velocities on X and Y axes

#### State Transition Model

The filter uses a constant velocity model with state transition matrix:
```
F = [1  0  dt  0 ]
    [0  1  0  dt]
    [0  0  1   0 ]
    [0  0  0   1 ]
```

Where `dt` is the time delta between measurements (~2ms for typical loop timing).

#### Measurement Model

The measurement matrix observes only position:
```
H = [1  0  0  0]  (measures Tx)
    [0  1  0  0]  (measures Ty)
```

### Combined Movement Logic

The firmware implements a two-stage decision process:

1. **Combined Tx/Ty Movement Detection**
   - Calculate magnitude: `mag_tx_ty = sqrt(tx² + ty²)`
   - If `mag_tx_ty > CONFIG_TXTY_COMBINED_THRESHOLD`:
     - Check individual deadzones: `|tx| > CONFIG_TX_DEADZONE OR |ty| > CONFIG_TY_DEADZONE`
     - If true, send both Tx and Ty values scaled appropriately
     - Return early (skip predominant movement logic)

2. **Fallback to Predominant Movement**
   - If combined threshold not met, use traditional predominant axis selection
   - Applies to all 6 DOF axes (Tx, Ty, Tz, Rx, Ry, Rz)
   - Maintains existing behavior for other axes

## Configuration Parameters

All parameters can be adjusted in `UserConfig.h`:

### Enable/Disable Feature

```cpp
// Set to true to enable combined Tx/Ty movements, false for traditional mode
#define CONFIG_ENABLE_TXTY_COMBINED  true
```

### Combined Movement Threshold

```cpp
// Minimum magnitude for combined Tx/Ty movements (raw sensor units)
// Default: 2.0
// Lower = more sensitive to diagonal movements
// Higher = only strong diagonal movements detected
#define CONFIG_TXTY_COMBINED_THRESHOLD  2.0
```

### 2D Kalman Filter Tuning

```cpp
// Position process noise (affects smoothness)
// Default: 0.01
// Lower = smoother but slower response
// Higher = faster response but less filtering
#define CONFIG_KALMAN2D_Q_POS  0.01

// Velocity process noise (affects acceleration response)
// Default: 0.1
// Lower = assumes constant velocity
// Higher = allows rapid velocity changes
#define CONFIG_KALMAN2D_Q_VEL  0.1

// Measurement noise (affects sensor trust)
// Default: 0.1
// Lower = trust sensor readings more
// Higher = apply more filtering
#define CONFIG_KALMAN2D_R  0.1
```

## Behavior Modes

### Combined Mode Enabled (`CONFIG_ENABLE_TXTY_COMBINED = true`)

- **Diagonal movements**: Both Tx and Ty sent when magnitude exceeds threshold
- **Single-axis movements**: Fall back to predominant axis if below threshold
- **Other axes (Tz, Rx, Ry, Rz)**: Always use predominant movement logic

### Combined Mode Disabled (`CONFIG_ENABLE_TXTY_COMBINED = false`)

- All axes use traditional predominant movement selection
- Tx/Ty filtered with independent 1D Kalman filters
- Maintains backward compatibility with original firmware

## Tuning Recommendations

### For Precise Diagonal Movements

```cpp
#define CONFIG_TXTY_COMBINED_THRESHOLD  1.5  // More sensitive
#define CONFIG_KALMAN2D_Q_POS  0.005         // Smoother
#define CONFIG_KALMAN2D_R  0.15              // More filtering
```

### For Responsive Single-Axis Movements

```cpp
#define CONFIG_TXTY_COMBINED_THRESHOLD  3.0  // Less sensitive to diagonals
#define CONFIG_KALMAN2D_Q_POS  0.02          // Faster response
#define CONFIG_KALMAN2D_R  0.08              // Less filtering
```

### For Noisy Environment

```cpp
#define CONFIG_KALMAN2D_Q_POS  0.005   // Smoother tracking
#define CONFIG_KALMAN2D_Q_VEL  0.05    // Less velocity noise
#define CONFIG_KALMAN2D_R  0.2         // More measurement noise filtering
```

## Testing Procedures

### 1. Combined Movement Test

1. Enable debug mode: `#define DEBUG_MODE true`
2. Move knob diagonally (e.g., forward-right)
3. Verify both Tx and Ty values are sent simultaneously
4. Check magnitude threshold is being applied correctly

### 2. Threshold Verification

1. Make small diagonal movements
2. Gradually increase movement amplitude
3. Observe transition point where combined mode activates
4. Adjust `CONFIG_TXTY_COMBINED_THRESHOLD` as needed

### 3. Single-Axis Verification

1. Move knob purely along X axis
2. Verify it falls back to predominant mode (only Tx sent)
3. Repeat for Y axis
4. Ensure no cross-talk between axes

### 4. Kalman Filter Response

1. Make quick diagonal movements
2. Observe smoothness vs responsiveness trade-off
3. Adjust Q and R parameters to optimize feel
4. Test with different movement speeds

## Performance Considerations

### Memory Usage

- 2D Kalman filter adds ~200 bytes (4x4 covariance matrix + state)
- Minimal impact on RP2040 with 264KB RAM

### Computational Load

- 2D filter update: ~50-100 additional floating-point operations per cycle
- Negligible impact on 133MHz RP2040 with 2ms loop time
- No blocking operations

### HID Throughput

- Combined mode may send more frequent HID reports for diagonal movements
- USB polling rate remains at 2ms (500Hz)
- No impact on button response or LED updates

## Integration with Existing Features

### Sensor Fusion

- Works seamlessly with dual magnetometer setup
- 2D filter receives fused Tx/Ty values from both sensors
- Maintains calibration and coordinate transformation

### Watchdog

- No changes to sensor watchdog behavior
- Reset logic operates before Kalman filtering

### LED Feedback

- `totalMove` calculation includes filtered Tx/Ty values
- Reactive LED mode responds to combined movements

### Other Axes

- Tz, Rx, Ry, Rz maintain existing Kalman filtering
- No changes to rotation axis discrimination
- Asymmetric scaling still applied

## Troubleshooting

### Issue: No combined movements detected

**Solutions:**
- Lower `CONFIG_TXTY_COMBINED_THRESHOLD`
- Check individual deadzone settings
- Verify `CONFIG_ENABLE_TXTY_COMBINED` is `true`

### Issue: Too sensitive to noise

**Solutions:**
- Increase `CONFIG_KALMAN2D_R` (more measurement noise filtering)
- Decrease `CONFIG_KALMAN2D_Q_POS` (smoother position tracking)
- Raise `CONFIG_TXTY_COMBINED_THRESHOLD`

### Issue: Sluggish response

**Solutions:**
- Increase `CONFIG_KALMAN2D_Q_POS` (faster adaptation)
- Decrease `CONFIG_KALMAN2D_R` (trust measurements more)
- Check loop timing (should be ~2ms)

### Issue: Unwanted diagonal movements

**Solutions:**
- Increase `CONFIG_TXTY_COMBINED_THRESHOLD`
- Verify sensor calibration is accurate
- Check for mechanical play in mounting

## Future Enhancements

Potential improvements for future versions:

1. **Adaptive threshold**: Automatically adjust based on movement statistics
2. **Configurable time delta**: Auto-detect loop timing for `dt`
3. **Extended state**: Add acceleration terms for smoother tracking
4. **Multi-axis combined**: Extend to 3D (Tx/Ty/Tz) or rotation axes
5. **User profiles**: Preset configurations for different use cases

## Technical References

- Kalman Filter: Welch & Bishop (2001), "An Introduction to the Kalman Filter"
- State Space Models: Särkkä (2013), "Bayesian Filtering and Smoothing"
- Sensor Fusion: Groves (2013), "Principles of GNSS, Inertial, and Multisensor Integrated Navigation"

---

**Developed as part of the AdaSpace3D project**  
For support and community discussion: [Discord Server](http://dsc.gg/axiom3d)
