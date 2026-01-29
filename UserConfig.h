#ifndef USER_CONFIG_H
#define USER_CONFIG_H

// ===================================================================================
//   ADA SPACE 3D - USER CONFIGURATION
// ===================================================================================

// --- LED CONFIGURATION ---
// The firmware drives BOTH outputs simultaneously. Connect whichever you like.
// - Addressable Strip: GPIO 4
// - Simple LED:        GPIO 3

#define LED_MODE            2      // 0 = Static (Solid Color)
                                   // 1 = Breathing (Gently Fades)
                                   // 2 = Debug / Reactive (Status Colors + White Flash on Move)

// Settings for Addressable Strip (GPIO 4)
#define NUM_ADDRESSABLE_LEDS 4      // How many LEDs are in your strip?
#define LED_BRIGHTNESS       130    // 0 to 255 (Global brightness)

// Settings for Static/Breathing Modes (Ignored in Debug Mode)
// Set your preferred color (0-255)
#define LED_COLOR_R         0      // Red
#define LED_COLOR_G         0    // Green
#define LED_COLOR_B         255    // Blue (Cyan)

// --- PIN DEFINITIONS ---
#define MAG_POWER_PIN       15     

// Button Pins
#define BUTTON1_PIN         A0
#define BUTTON2_PIN         A1
#define BUTTON3_PIN         A2
#define BUTTON4_PIN         A3

// --- SENSOR SETTINGS ---
// Per-axis scaling values - fine-tune sensitivity for each axis independently
// Higher values = more sensitive (larger movements in software)
// Lower values = less sensitive (smaller movements in software)
#define CONFIG_TX_SCALE        100   // Translation X (left/right) sensitivity
#define CONFIG_TY_SCALE        100   // Translation Y (forward/back) sensitivity
#define CONFIG_TZ_SCALE        50    // Translation Z (up/down) sensitivity
#define CONFIG_RX_SCALE        40    // Rotation X (pitch) sensitivity
#define CONFIG_RY_SCALE        40    // Rotation Y (roll) sensitivity
#define CONFIG_RZ_SCALE        40    // Rotation Z (yaw) sensitivity

// Legacy grouped settings (kept for reference, not used)
// #define CONFIG_TRANS_SCALE     100 
// #define CONFIG_ZOOM_SCALE      50  
// #define CONFIG_ROT_SCALE       40 

// Asymmetric scaling multipliers (compensate for non-linear magnetic field)
// When magnets move AWAY from sensors, signal is weaker - these multipliers compensate
// Values > 1.0 increase sensitivity for that direction
// Set both to 1.0 for symmetric scaling
#define CONFIG_RX_POSITIVE_MULT  1.5   // Rx forward (magnet away) - typically needs boost
#define CONFIG_RX_NEGATIVE_MULT  1.0   // Rx backward (magnet closer)
#define CONFIG_RY_POSITIVE_MULT  1.5   // Ry left (magnet away) - typically needs boost
#define CONFIG_RY_NEGATIVE_MULT  1.0   // Ry right (magnet closer)
#define CONFIG_TZ_POSITIVE_MULT  1.0   // Tz up (magnet closer)
#define CONFIG_TZ_NEGATIVE_MULT  1.5   // Tz down (magnet away) - typically needs boost

// Per-axis deadzones - filter out noise and micro-movements for each axis independently
// Keep values small (1.0-2.5) because raw sensor values are tiny
// Higher values = less noise but less responsive
// Lower values = more responsive but more noise
#define CONFIG_TX_DEADZONE     1.0    // Translation X deadzone
#define CONFIG_TY_DEADZONE     1.0    // Translation Y deadzone
#define CONFIG_TZ_DEADZONE     2.5    // Translation Z deadzone
#define CONFIG_RX_DEADZONE     1.5    // Rotation X (pitch) deadzone
#define CONFIG_RY_DEADZONE     1.5    // Rotation Y (roll) deadzone
#define CONFIG_RZ_DEADZONE     1.5    // Rotation Z (yaw) deadzone

// Legacy grouped deadzones (kept for reference, not used)
// #define CONFIG_DEADZONE        1.0    // X/Y translation deadzone
// #define CONFIG_ZOOM_DEADZONE   2.5    // Z translation deadzone  
// #define CONFIG_ROT_DEADZONE    1.5    // Rotation deadzones

// Backward compatibility defines for legacy code (single sensor mode, LED handler)
// These provide default values based on per-axis settings for functions that still use grouped constants
#define CONFIG_DEADZONE        CONFIG_TX_DEADZONE     // Use TX deadzone as default for translations
#define CONFIG_ZOOM_DEADZONE   CONFIG_TZ_DEADZONE     // Use TZ deadzone for zoom
#define CONFIG_ROT_DEADZONE    CONFIG_RX_DEADZONE     // Use RX deadzone as default for rotations
#define CONFIG_TRANS_SCALE     CONFIG_TX_SCALE        // Use TX scale as default for translations
#define CONFIG_ZOOM_SCALE      CONFIG_TZ_SCALE        // Use TZ scale for zoom
#define CONFIG_ROT_SCALE       CONFIG_RX_SCALE        // Use RX scale as default for rotations

// Minimum movement threshold for dual sensor mode
// This determines the minimum movement value to be considered significant
// Lower values = more sensitive, Higher values = less responsive but less noise
// Adjust based on your physical setup and magnet distances (7-8mm default)
#define CONFIG_MIN_MOVEMENT_THRESHOLD  0.5

// Rotation axis discrimination threshold (dual sensor mode)
// Determines how much more one sensor must vary than the other to be considered
// the dominant axis (Rx vs Ry discrimination)
// 1.3 = sensor must vary 30% more than the other
// Higher values = stricter separation (less cross-talk but may miss mixed movements)
// Lower values = more sensitive (catches mixed movements but more cross-talk)
#define CONFIG_ROTATION_AXIS_RATIO  1.3

// --- USB IDENTIFICATION ---
// 0x046d / 0xc626 = SpaceNavigator (Best for DIY compatibility)
#define USB_VID             0x256f
#define USB_PID             0xc631

// --- DEBUG MODE ---
// MUST be false for normal use. True = Serial Monitor but NO Driver.
#define DEBUG_MODE          false

#endif // USER_CONFIG_H

