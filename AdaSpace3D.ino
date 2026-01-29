/*
 * AdaSpace3D - Unified Firmware (Dual Magnetometer Edition)
 * * Features: Dual LED Drive, Reactive Lighting, Auto-Hardware Detect, Dual Magnetometer Fusion
 * * Safety:   Includes Sensor Watchdog to auto-reset frozen I2C lines
 * * Advanced: Kalman Filtering, 6DOF separation, Predominant Movement Detection
 * 
 * Dual Magnetometer Configuration:
 * - Sensor 1 (Solder): Located at 3 o'clock position under knob
 * - Sensor 2 (Cable):  Located at 6 o'clock position, rotated 90° clockwise
 *   - Coordinate transformation applied: X' = Y, Y' = -X, Z' = Z
 * 
 * Sensor Fusion Logic:
 * - Translation (Tx, Ty, Tz): Average of both sensors
 * - Rotation (Rx, Ry, Rz): Differential measurements between sensors
 * - Kalman filtering applied to all 6 DOF for noise reduction
 * - Only predominant movement is transmitted via HID (no cross-talk)
 */

#include "Adafruit_TinyUSB.h"
#include "TLx493D_inc.hpp"
#include <Adafruit_NeoPixel.h>
#include "UserConfig.h"

// --- CONSTANTS ---
#ifdef PIN_NEOPIXEL
  #undef PIN_NEOPIXEL  // Undefine board variant default to use our custom pin
#endif
#define PIN_NEOPIXEL   4
#define PIN_SIMPLE     3
#define HANG_THRESHOLD 50              // consecutive identical readings before reset
#define PREVENTIVE_RESET_INTERVAL 0    // Set to 300000 (5 mins) if you want periodic resets
#define LED_MOVEMENT_INTENSITY_MULTIPLIER 30.0  // Converts movement magnitude to LED brightness

// --- LED SETUP ---
Adafruit_NeoPixel strip(NUM_ADDRESSABLE_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// --- HARDWARE GLOBALS ---
const uint8_t physPins[] = {BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN, BUTTON4_PIN};
const uint8_t physToHID[] = {13, 14, 15, 16};
bool currentButtonState[] = {false, false, false, false};
bool prevButtonState[] = {false, false, false, false};

// --- DATA STRUCTURES ---
struct MagCalibration {
  double x_neutral = 0.0, y_neutral = 0.0, z_neutral = 0.0;
  bool calibrated = false;
};

// Kalman filter for a single axis
struct KalmanFilter {
  double q = 0.01;     // Process noise covariance
  double r = 0.1;      // Measurement noise covariance
  double x = 0.0;      // Estimated value
  double p = 1.0;      // Estimation error covariance
  double k = 0.0;      // Kalman gain
  
  double update(double measurement) {
    // Prediction
    p = p + q;
    
    // Prevent filter lock-up by maintaining minimum covariance
    if (p < 0.001) p = 0.001;
    
    // Update
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;
    
    return x;
  }
};

// 2D Kalman filter for combined Tx/Ty movements
// State: [Tx, Ty, Vx, Vy] where Vx, Vy are velocities
struct KalmanFilter2D {
  // State vector: [Tx, Ty, Vx, Vy]
  double state[4] = {0.0, 0.0, 0.0, 0.0};
  
  // State covariance matrix P (4x4) - stored as 1D array
  double P[16];
  
  // Process noise covariance Q
  double q_pos = 0.01;   // Position process noise
  double q_vel = 0.1;    // Velocity process noise
  
  // Measurement noise covariance R
  double r = 0.1;
  
  // Time delta (in seconds) - default for ~2ms loop
  double dt = 0.002;
  
  KalmanFilter2D() {
    // Initialize covariance matrix P as identity
    for(int i = 0; i < 16; i++) P[i] = 0.0;
    P[0] = 1.0;  // P(0,0)
    P[5] = 1.0;  // P(1,1)
    P[10] = 1.0; // P(2,2)
    P[15] = 1.0; // P(3,3)
  }
  
  void update(double meas_tx, double meas_ty) {
    // State transition matrix F:
    // [1  0  dt  0]
    // [0  1  0  dt]
    // [0  0  1   0]
    // [0  0  0   1]
    
    // --- PREDICTION STEP ---
    // Predict state: x_pred = F * x
    double pred_tx = state[0] + state[2] * dt;
    double pred_ty = state[1] + state[3] * dt;
    double pred_vx = state[2];
    double pred_vy = state[3];
    
    // Predict covariance: P_pred = F * P * F' + Q
    // Simplified update for efficiency (only updating diagonal and dt-related terms)
    double P_pred[16];
    for(int i = 0; i < 16; i++) P_pred[i] = P[i];
    
    // Add process noise to position and velocity covariances
    P_pred[0] += q_pos + q_vel * dt * dt;  // P(0,0) - Tx variance
    P_pred[5] += q_pos + q_vel * dt * dt;  // P(1,1) - Ty variance
    P_pred[10] += q_vel;                    // P(2,2) - Vx variance
    P_pred[15] += q_vel;                    // P(3,3) - Vy variance
    
    // Cross-correlation terms (position-velocity coupling)
    P_pred[2] += P[10] * dt;   // P(0,2)
    P_pred[8] = P_pred[2];     // P(2,0) - symmetric
    P_pred[7] += P[15] * dt;   // P(1,3)
    P_pred[13] = P_pred[7];    // P(3,1) - symmetric
    
    // Prevent covariance collapse
    if(P_pred[0] < 0.001) P_pred[0] = 0.001;
    if(P_pred[5] < 0.001) P_pred[5] = 0.001;
    if(P_pred[10] < 0.001) P_pred[10] = 0.001;
    if(P_pred[15] < 0.001) P_pred[15] = 0.001;
    
    // --- UPDATE STEP ---
    // Measurement matrix H:
    // [1  0  0  0]  - We measure Tx
    // [0  1  0  0]  - We measure Ty
    
    // Innovation (measurement residual): y = z - H * x_pred
    double innov_tx = meas_tx - pred_tx;
    double innov_ty = meas_ty - pred_ty;
    
    // Innovation covariance: S = H * P_pred * H' + R
    double S_tx = P_pred[0] + r;  // S(0,0)
    double S_ty = P_pred[5] + r;  // S(1,1)
    
    // Kalman gain: K = P_pred * H' * inv(S)
    // For 2D case with independent measurements, this simplifies:
    double K_tx[4], K_ty[4];
    K_tx[0] = P_pred[0] / S_tx;   // K(0,0)
    K_tx[1] = P_pred[4] / S_tx;   // K(1,0)
    K_tx[2] = P_pred[8] / S_tx;   // K(2,0)
    K_tx[3] = P_pred[12] / S_tx;  // K(3,0)
    
    K_ty[0] = P_pred[1] / S_ty;   // K(0,1)
    K_ty[1] = P_pred[5] / S_ty;   // K(1,1)
    K_ty[2] = P_pred[9] / S_ty;   // K(2,1)
    K_ty[3] = P_pred[13] / S_ty;  // K(3,1)
    
    // Update state: x = x_pred + K * y
    state[0] = pred_tx + K_tx[0] * innov_tx + K_ty[0] * innov_ty;
    state[1] = pred_ty + K_tx[1] * innov_tx + K_ty[1] * innov_ty;
    state[2] = pred_vx + K_tx[2] * innov_tx + K_ty[2] * innov_ty;
    state[3] = pred_vy + K_tx[3] * innov_tx + K_ty[3] * innov_ty;
    
    // Update covariance: P = (I - K * H) * P_pred
    // Simplified: P = P_pred - K * S * K'
    for(int i = 0; i < 16; i++) P[i] = P_pred[i];
    
    P[0] -= K_tx[0] * S_tx * K_tx[0] + K_ty[0] * S_ty * K_ty[0];
    P[5] -= K_tx[1] * S_tx * K_tx[1] + K_ty[1] * S_ty * K_ty[1];
    P[10] -= K_tx[2] * S_tx * K_tx[2] + K_ty[2] * S_ty * K_ty[2];
    P[15] -= K_tx[3] * S_tx * K_tx[3] + K_ty[3] * S_ty * K_ty[3];
  }
  
  void getPosition(double* tx, double* ty) {
    *tx = state[0];
    *ty = state[1];
  }
  
  void getVelocity(double* vx, double* vy) {
    *vx = state[2];
    *vy = state[3];
  }
};

// Calibration for both sensors
MagCalibration magCalSolder, magCalCable;

struct SensorWatchdog {
  double last_x = 0.0, last_y = 0.0, last_z = 0.0;
  int sameValueCount = 0;
  unsigned long lastResetTime = 0;
  unsigned long lastPreventiveReset = 0;
};

SensorWatchdog watchdogSolder, watchdogCable;

// Kalman filters for 6 DOF (Translation and Rotation)
KalmanFilter kalman_tx, kalman_ty, kalman_tz;
KalmanFilter kalman_rx, kalman_ry, kalman_rz;

// 2D Kalman filter for combined Tx/Ty movements
KalmanFilter2D kalman_2d_txty;

using namespace ifx::tlx493d;

TLx493D_A1B6 magCable(Wire1, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6 magSolder(Wire, TLx493D_IIC_ADDR_A0_e);
bool bothSensorsActive = false;
bool cableSensorActive = false;

// HID Report Descriptor
static const uint8_t spaceMouse_hid_report_desc[] = {
  0x05, 0x01, 0x09, 0x08, 0xA1, 0x01, 0xA1, 0x00, 0x85, 0x01, 0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F, 0x36, 0x00, 0x80, 0x46, 0xFF, 0x7F,
  0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02, 0xC0,
  0xA1, 0x00, 0x85, 0x02, 0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F, 0x36, 0x00, 0x80, 0x46, 0xFF, 0x7F,
  0x09, 0x33, 0x09, 0x34, 0x09, 0x35, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02, 0xC0,
  0xA1, 0x00, 0x85, 0x03, 0x05, 0x09, 0x19, 0x01, 0x29, 0x20, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x01, 0x95, 0x20, 0x81, 0x02, 0xC0,
  0xC0
};

Adafruit_USBD_HID usb_hid;
#define CALIBRATION_SAMPLES 50

// --- LED LOGIC ---

void updateHardwareLeds(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t c = strip.Color(r, g, b);
  strip.fill(c);
  strip.show();

  int brightness = (r * 77 + g * 150 + b * 29) >> 8; 
  brightness = (brightness * LED_BRIGHTNESS) / 255;
  analogWrite(PIN_SIMPLE, brightness);
}

void blinkError() {
  while(1) {
    updateHardwareLeds(255, 0, 0); delay(100);
    updateHardwareLeds(0, 0, 0);   delay(100);
  }
}

void handleLeds(double totalMove) {
  if (LED_MODE == 0) { // STATIC
    // Rate-limit: static color doesn't need updating every cycle
    static unsigned long lastStaticUpdate = 0;
    if (millis() - lastStaticUpdate < 500) return; // Only refresh every 500ms
    lastStaticUpdate = millis();
    updateHardwareLeds(LED_COLOR_R, LED_COLOR_G, LED_COLOR_B);
  }
  else if (LED_MODE == 1) { // BREATHING
    float val = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
    uint8_t r = (LED_COLOR_R * (int)val) / 255;
    uint8_t g = (LED_COLOR_G * (int)val) / 255;
    uint8_t b = (LED_COLOR_B * (int)val) / 255;
    updateHardwareLeds(r, g, b);
  }
  else if (LED_MODE == 2) { // REACTIVE
      // Rate-limit LED updates to prevent strip.show() from blocking HID reports
      static unsigned long lastLedUpdate = 0;
      if (millis() - lastLedUpdate < 50) return; // Only update LEDs every 50ms
      lastLedUpdate = millis();
      
      int minScale = 50;  
      int maxScale = 255; 
      int currentScale = minScale;

      if (totalMove > CONFIG_DEADZONE) {
         int addedIntensity = (int)(totalMove * LED_MOVEMENT_INTENSITY_MULTIPLIER);
         currentScale = constrain(minScale + addedIntensity, minScale, maxScale);
      }
      
      uint8_t r = (LED_COLOR_R * currentScale) / 255;
      uint8_t g = (LED_COLOR_G * currentScale) / 255;
      uint8_t b = (LED_COLOR_B * currentScale) / 255;
      updateHardwareLeds(r, g, b);
  }
}

void resetMagnetometer(bool isCable) {
  // Briefly flash Red to indicate reset
  updateHardwareLeds(255, 0, 0);
  
  TLx493D_A1B6* sensor = isCable ? &magCable : &magSolder;
  SensorWatchdog* wd = isCable ? &watchdogCable : &watchdogSolder;
  
  sensor->end();
  delay(50);
  
  // Restart the correct Wire interface
  if (isCable) {
      Wire1.end(); delay(50); Wire1.begin(); 
  } else {
      Wire.end(); delay(50); Wire.begin();
  }
  delay(50);
  
  // Try to reinitialize the sensor
  bool success = sensor->begin();
  
  if (success) {
    wd->sameValueCount = 0;
    wd->lastResetTime = millis();
    // Return to normal color
    updateHardwareLeds(LED_COLOR_R, LED_COLOR_G, LED_COLOR_B);
  } else {
    // Reset failed - flash red LED as error indicator
    for (int i = 0; i < 3; i++) {
      updateHardwareLeds(255, 0, 0); delay(100);
      updateHardwareLeds(0, 0, 0); delay(100);
    }
    
    // If in dual sensor mode and one fails, degrade to single sensor mode
    if (bothSensorsActive) {
      bothSensorsActive = false;
      cableSensorActive = !isCable;  // Use the other sensor
      updateHardwareLeds(255, 128, 0); delay(500); // Orange = degraded mode
    }
  }
}

void setup() {
  pinMode(PIN_SIMPLE, OUTPUT);
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  updateHardwareLeds(255, 0, 0); // Boot Red

#if DEBUG_MODE
  Serial.begin(115200);
#endif

  for(uint8_t i = 0; i < 4; i++) pinMode(physPins[i], INPUT_PULLUP);

  TinyUSBDevice.setID(USB_VID, USB_PID);
  usb_hid.setReportDescriptor(spaceMouse_hid_report_desc, sizeof(spaceMouse_hid_report_desc));
  usb_hid.setPollInterval(2);
  usb_hid.begin();

  while(!TinyUSBDevice.mounted()) delay(100);
  
  pinMode(MAG_POWER_PIN, OUTPUT);
  digitalWrite(MAG_POWER_PIN, HIGH);
  delay(10); 

  // Try to initialize both sensors
  Wire1.begin(); Wire1.setClock(400000);
  Wire.begin(); Wire.setClock(400000);
  
  bool cable_ok = magCable.begin();
  bool solder_ok = magSolder.begin();
  
  if (cable_ok && solder_ok) {
    bothSensorsActive = true;
    cableSensorActive = true;
    updateHardwareLeds(0, 0, 255); delay(500); // Blue for Dual Sensor mode
  } 
  else if (cable_ok) {
    bothSensorsActive = false;
    cableSensorActive = true;
    updateHardwareLeds(0, 255, 0); delay(500); // Green for Cable only
  } 
  else if (solder_ok) {
    bothSensorsActive = false;
    cableSensorActive = false;
    updateHardwareLeds(0, 255, 255); delay(500); // Cyan for Solder only
  } 
  else {
    blinkError(); 
  }

  watchdogSolder.lastPreventiveReset = millis();
  watchdogCable.lastPreventiveReset = millis();
  calibrateMagnetometer();
}

void loop() {
  for(uint8_t i = 0; i < 4; i++) {
    currentButtonState[i] = (digitalRead(physPins[i]) == LOW);
    if (currentButtonState[i] != prevButtonState[i]) {
      setButtonStateHID(physToHID[i], currentButtonState[i]);
      prevButtonState[i] = currentButtonState[i];
    }
  }

  readAndSendMagnetometerData();
  delay(2);
}

void calibrateMagnetometer() {
  double sumX_solder = 0, sumY_solder = 0, sumZ_solder = 0;
  double sumX_cable = 0, sumY_cable = 0, sumZ_cable = 0;
  int valid_solder = 0, valid_cable = 0;
  
  updateHardwareLeds(0, 0, 0); delay(100);
  updateHardwareLeds(255, 255, 255); 
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    double x, y, z;
    
    // Calibrate solder sensor (if active)
    if (!cableSensorActive || bothSensorsActive) {
      if(magSolder.getMagneticField(&x, &y, &z)) {
        sumX_solder += x; sumY_solder += y; sumZ_solder += z;
        valid_solder++;
      }
    }
    
    // Calibrate cable sensor (if active)
    if(cableSensorActive && magCable.getMagneticField(&x, &y, &z)) {
      sumX_cable += x; sumY_cable += y; sumZ_cable += z;
      valid_cable++;
    }
    
    delay(10);
  }

  // Update calibration for solder sensor
  if (!cableSensorActive || bothSensorsActive) {
    if(valid_solder > 0) {
      magCalSolder.x_neutral = sumX_solder / valid_solder;
      magCalSolder.y_neutral = sumY_solder / valid_solder;
      magCalSolder.z_neutral = sumZ_solder / valid_solder;
      magCalSolder.calibrated = true;
    }
  }
  
  // Update calibration for cable sensor
  if(cableSensorActive && valid_cable > 0) {
    magCalCable.x_neutral = sumX_cable / valid_cable;
    magCalCable.y_neutral = sumY_cable / valid_cable;
    magCalCable.z_neutral = sumZ_cable / valid_cable;
    magCalCable.calibrated = true;
  }
  
  // Check if calibration succeeded
  bool calibration_success = false;
  if (bothSensorsActive) {
    calibration_success = (valid_solder > 0 && valid_cable > 0);
  } else if (cableSensorActive) {
    calibration_success = (valid_cable > 0);
  } else {
    calibration_success = (valid_solder > 0);
  }
  
  if(calibration_success) {
    updateHardwareLeds(0, 0, 0); delay(200);
  } else {
    for(int i=0; i<5; i++) {
       updateHardwareLeds(255, 0, 0); delay(50);
       updateHardwareLeds(0, 0, 0); delay(50);
    }
    calibrateMagnetometer();
  }
}

void readAndSendMagnetometerData() {
  // If only one sensor, fall back to legacy behavior
  if (!bothSensorsActive) {
    readAndSendSingleMagnetometer();
    return;
  }
  
  // Read both sensors
  double x_solder, y_solder, z_solder;
  double x_cable, y_cable, z_cable;
  
  bool solder_ok = magSolder.getMagneticField(&x_solder, &y_solder, &z_solder);
  bool cable_ok = magCable.getMagneticField(&x_cable, &y_cable, &z_cable);
  
  if (!solder_ok || !cable_ok) {
    send_tx_rx_reports(0, 0, 0, 0, 0, 0);
    return;
  }
  
  // Check for finite values
  if (!isfinite(x_solder) || !isfinite(y_solder) || !isfinite(z_solder) ||
      !isfinite(x_cable) || !isfinite(y_cable) || !isfinite(z_cable)) {
    send_tx_rx_reports(0, 0, 0, 0, 0, 0);
    return;
  }
  
  // Watchdog for solder sensor
  if (x_solder == watchdogSolder.last_x && y_solder == watchdogSolder.last_y && z_solder == watchdogSolder.last_z) {
    watchdogSolder.sameValueCount++;
    if (watchdogSolder.sameValueCount >= HANG_THRESHOLD) {
      resetMagnetometer(false);
      return;
    }
  } else {
    watchdogSolder.sameValueCount = 0;
    watchdogSolder.last_x = x_solder;
    watchdogSolder.last_y = y_solder;
    watchdogSolder.last_z = z_solder;
  }
  
  // Watchdog for cable sensor
  if (x_cable == watchdogCable.last_x && y_cable == watchdogCable.last_y && z_cable == watchdogCable.last_z) {
    watchdogCable.sameValueCount++;
    if (watchdogCable.sameValueCount >= HANG_THRESHOLD) {
      resetMagnetometer(true);
      return;
    }
  } else {
    watchdogCable.sameValueCount = 0;
    watchdogCable.last_x = x_cable;
    watchdogCable.last_y = y_cable;
    watchdogCable.last_z = z_cable;
  }
  
  // Apply calibration
  if (magCalSolder.calibrated) {
    x_solder -= magCalSolder.x_neutral;
    y_solder -= magCalSolder.y_neutral;
    z_solder -= magCalSolder.z_neutral;
  }
  
  if (magCalCable.calibrated) {
    x_cable -= magCalCable.x_neutral;
    y_cable -= magCalCable.y_neutral;
    z_cable -= magCalCable.z_neutral;
  }
  
  // Transform cable sensor coordinates (rotated 90° clockwise)
  // X' = Y, Y' = -X, Z' = Z
  double x_cable_transformed = y_cable;
  double y_cable_transformed = -x_cable;
  double z_cable_transformed = z_cable;
  
  // Calculate 6 DOF from dual sensor fusion
  // Translation: average of both sensors for linear movement
  double raw_tx = (x_solder + x_cable_transformed) / 2.0;
  double raw_ty = (y_solder + y_cable_transformed) / 2.0;
  double raw_tz = (z_solder + z_cable_transformed) / 2.0;
  
  // Rotation: Context-aware differential measurements (Version 4)
  // Key insight: After calibration, z_solder and z_cable_transformed are deviations from neutral
  // When knob tilts, ONE sensor shows significantly more Z-axis change than the other
  // Solder at 3 o'clock (right), Cable at 6 o'clock (bottom after transform)
  // 
  // New approach based on user feedback: evaluate which sensor varies more
  // - If cable (bottom) varies more → Pitch (Rx) - forward/back tilt
  // - If solder (right) varies more → Roll (Ry) - left/right tilt
  
  double z_solder_abs = abs(z_solder);
  double z_cable_abs = abs(z_cable_transformed);
  
  // Determine which sensor shows more variation and assign to appropriate axis
  double raw_rx = 0.0;
  double raw_ry = 0.0;
  
  if (z_cable_abs > z_solder_abs * CONFIG_ROTATION_AXIS_RATIO) {
    // Cable sensor (bottom) varies significantly more → Pitch (forward/back tilt)
    raw_rx = z_cable_transformed;  // Use cable's Z deviation for pitch
    raw_ry = 0.0;  // Suppress roll
  } 
  else if (z_solder_abs > z_cable_abs * CONFIG_ROTATION_AXIS_RATIO) {
    // Solder sensor (right) varies significantly more → Roll (left/right tilt)
    raw_rx = 0.0;  // Suppress pitch
    raw_ry = z_solder;  // Use solder's Z deviation for roll
  }
  else {
    // Both sensors vary similarly - use differential (ambiguous case)
    // This maintains some response when movement is not clearly pitch or roll
    raw_rx = (z_solder - z_cable_transformed) * 0.5;  // Reduced sensitivity for ambiguous case
    raw_ry = (z_cable_transformed - z_solder) * 0.5;
  }
  
  double raw_rz = (x_solder - y_solder) - (x_cable_transformed - y_cable_transformed);  // Yaw: XY asymmetry difference
  
  // Apply Kalman filtering
  // Use 2D Kalman filter for Tx/Ty if combined mode is enabled
  double tx, ty;
  if (CONFIG_ENABLE_TXTY_COMBINED) {
    // Configure 2D Kalman filter with user parameters
    kalman_2d_txty.q_pos = CONFIG_KALMAN2D_Q_POS;
    kalman_2d_txty.q_vel = CONFIG_KALMAN2D_Q_VEL;
    kalman_2d_txty.r = CONFIG_KALMAN2D_R;
    
    // Update 2D filter with Tx/Ty measurements
    kalman_2d_txty.update(raw_tx, raw_ty);
    kalman_2d_txty.getPosition(&tx, &ty);
  } else {
    // Fall back to independent 1D filters
    tx = kalman_tx.update(raw_tx);
    ty = kalman_ty.update(raw_ty);
  }
  
  double tz = kalman_tz.update(raw_tz);
  double rx = kalman_rx.update(raw_rx);
  double ry = kalman_ry.update(raw_ry);
  double rz = kalman_rz.update(raw_rz);
  
  // Calculate total movement for LED feedback
  double totalMove = abs(tx) + abs(ty) + abs(tz) + abs(rx) + abs(ry) + abs(rz);
  handleLeds(totalMove);
  
  // NEW LOGIC: Check for combined Tx/Ty movement first
  if (CONFIG_ENABLE_TXTY_COMBINED) {
    // Calculate combined magnitude of Tx and Ty
    double mag_tx_ty = sqrt(tx * tx + ty * ty);
    
    // If combined magnitude exceeds threshold, send both Tx and Ty
    if (mag_tx_ty > CONFIG_TXTY_COMBINED_THRESHOLD) {
      // Check that at least one of Tx or Ty exceeds its individual deadzone
      if (abs(tx) > CONFIG_TX_DEADZONE || abs(ty) > CONFIG_TY_DEADZONE) {
        // Scale and send combined Tx/Ty movement
        int16_t out_tx = (int16_t)constrain(-tx * CONFIG_TX_SCALE, -32767, 32767);
        int16_t out_ty = (int16_t)constrain(-ty * CONFIG_TY_SCALE, -32767, 32767);
        send_tx_rx_reports(out_tx, out_ty, 0, 0, 0, 0);
        return;
      }
    }
  }
  
  // FALLBACK: Use predominant movement logic for all axes
  // Find predominant movement BEFORE applying deadzones
  double movements[6] = {abs(tx), abs(ty), abs(tz), abs(rx), abs(ry), abs(rz)};
  int maxIdx = 0;
  double maxVal = movements[0];
  
  for (int i = 1; i < 6; i++) {
    if (movements[i] > maxVal) {
      maxVal = movements[i];
      maxIdx = i;
    }
  }
  
  // Apply deadzones based on movement type (per-axis configuration)
  double* values[6] = {&tx, &ty, &tz, &rx, &ry, &rz};
  double thresholds[6] = {CONFIG_TX_DEADZONE, CONFIG_TY_DEADZONE, CONFIG_TZ_DEADZONE, 
                          CONFIG_RX_DEADZONE, CONFIG_RY_DEADZONE, CONFIG_RZ_DEADZONE};
  
  // Check if predominant movement is above its threshold
  if (abs(*values[maxIdx]) < thresholds[maxIdx] || maxVal < CONFIG_MIN_MOVEMENT_THRESHOLD) {
    send_tx_rx_reports(0, 0, 0, 0, 0, 0);
    return;
  }
  
  // Scale and send only predominant movement
  // Apply per-axis scaling with asymmetric multipliers to compensate for non-linear magnetic field
  int16_t out_tx = 0, out_ty = 0, out_tz = 0;
  int16_t out_rx = 0, out_ry = 0, out_rz = 0;
  
  switch (maxIdx) {
    case 0: // Tx
      out_tx = (int16_t)constrain(-tx * CONFIG_TX_SCALE, -32767, 32767);
      break;
    case 1: // Ty
      out_ty = (int16_t)constrain(-ty * CONFIG_TY_SCALE, -32767, 32767);
      break;
    case 2: // Tz - asymmetric scaling
      if (tz >= 0) {
        // Positive (up/closer) - use positive multiplier
        out_tz = (int16_t)constrain(tz * CONFIG_TZ_SCALE * CONFIG_TZ_POSITIVE_MULT, -32767, 32767);
      } else {
        // Negative (down/farther) - use negative multiplier
        out_tz = (int16_t)constrain(tz * CONFIG_TZ_SCALE * CONFIG_TZ_NEGATIVE_MULT, -32767, 32767);
      }
      break;
    case 3: // Rx - asymmetric scaling
      if (rx >= 0) {
        // Positive (forward/farther) - use positive multiplier
        out_rx = (int16_t)constrain(rx * CONFIG_RX_SCALE * CONFIG_RX_POSITIVE_MULT, -32767, 32767);
      } else {
        // Negative (backward/closer) - use negative multiplier
        out_rx = (int16_t)constrain(rx * CONFIG_RX_SCALE * CONFIG_RX_NEGATIVE_MULT, -32767, 32767);
      }
      break;
    case 4: // Ry - asymmetric scaling
      if (ry >= 0) {
        // Positive (left/farther) - use positive multiplier
        out_ry = (int16_t)constrain(ry * CONFIG_RY_SCALE * CONFIG_RY_POSITIVE_MULT, -32767, 32767);
      } else {
        // Negative (right/closer) - use negative multiplier
        out_ry = (int16_t)constrain(ry * CONFIG_RY_SCALE * CONFIG_RY_NEGATIVE_MULT, -32767, 32767);
      }
      break;
    case 5: // Rz
      out_rz = (int16_t)constrain(rz * CONFIG_RZ_SCALE, -32767, 32767);
      break;
  }
  
  send_tx_rx_reports(out_tx, out_ty, out_tz, out_rx, out_ry, out_rz);
}

// Legacy single sensor function
void readAndSendSingleMagnetometer() {
  double x, y, z;
  TLx493D_A1B6* activeSensor = cableSensorActive ? &magCable : &magSolder;
  SensorWatchdog* watchdog = cableSensorActive ? &watchdogCable : &watchdogSolder;
  MagCalibration* magCal = cableSensorActive ? &magCalCable : &magCalSolder;
  
  // Preventive Reset Check
  if(PREVENTIVE_RESET_INTERVAL > 0 && millis() - watchdog->lastPreventiveReset > PREVENTIVE_RESET_INTERVAL) {
    resetMagnetometer(cableSensorActive);
    watchdog->lastPreventiveReset = millis();
    return;
  }

  if(activeSensor->getMagneticField(&x, &y, &z)) {
      if(isfinite(x) && isfinite(y) && isfinite(z)) {
          
          // --- WATCHDOG LOGIC ---
          if(x == watchdog->last_x && y == watchdog->last_y && z == watchdog->last_z) {
            watchdog->sameValueCount++;
            if(watchdog->sameValueCount >= HANG_THRESHOLD) {
              resetMagnetometer(cableSensorActive);
              return;
            }
          } else {
            watchdog->sameValueCount = 0;
            watchdog->last_x = x;
            watchdog->last_y = y;
            watchdog->last_z = z;
          }

          // Calibration
          if(magCal->calibrated) {
            x -= magCal->x_neutral;
            y -= magCal->y_neutral;
            z -= magCal->z_neutral;
          }
          
          double totalMove = abs(x) + abs(y) + abs(z);
          handleLeds(totalMove);

          if(abs(x) < CONFIG_DEADZONE) x = 0.0;
          if(abs(y) < CONFIG_DEADZONE) y = 0.0;
          if(abs(z) < CONFIG_ZOOM_DEADZONE) z = 0.0;

          int16_t tx = (int16_t)constrain(-x * CONFIG_TRANS_SCALE, -32767, 32767);
          int16_t ty = (int16_t)constrain(-y * CONFIG_TRANS_SCALE, -32767, 32767);
          int16_t tz = (int16_t)constrain(z * CONFIG_ZOOM_SCALE, -32767, 32767);
          int16_t rx = (int16_t)constrain(y * CONFIG_ROT_SCALE, -32767, 32767);
          int16_t ry = (int16_t)constrain(x * CONFIG_ROT_SCALE, -32767, 32767);
          
          send_tx_rx_reports(tx, ty, tz, rx, ry, 0);
      }
  } else {
      send_tx_rx_reports(0, 0, 0, 0, 0, 0);
  }
}

void setButtonStateHID(uint8_t hidButton, bool pressed) {
  if (!TinyUSBDevice.mounted()) return;
  uint8_t report[4] = {0, 0, 0, 0};
  if (pressed && hidButton <= 32) {
    report[(hidButton - 1) / 8] = (1 << ((hidButton - 1) % 8));
  }
  if (usb_hid.ready()) usb_hid.sendReport(3, report, 4);
}

void send_tx_rx_reports(int16_t tx, int16_t ty, int16_t tz, int16_t rx, int16_t ry, int16_t rz) {
  if (!TinyUSBDevice.mounted() || !usb_hid.ready()) return;

  tx = -tx;
  tz = -tz;
  rx = -rx;

  uint8_t tx_report[6] = {(uint8_t)tx, (uint8_t)(tx>>8), (uint8_t)ty, (uint8_t)(ty>>8), (uint8_t)tz, (uint8_t)(tz>>8)};
  usb_hid.sendReport(1, tx_report, 6);
  
  delayMicroseconds(500);
  
  uint8_t rx_report[6] = {(uint8_t)rx, (uint8_t)(rx>>8), (uint8_t)ry, (uint8_t)(ry>>8), (uint8_t)rz, (uint8_t)(rz>>8)};
  usb_hid.sendReport(2, rx_report, 6);
}

