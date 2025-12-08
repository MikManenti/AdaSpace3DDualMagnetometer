/*
 * AdaSpace3D - Unified Firmware (Reactive Color)
 * * Mode 2: "Dim-to-Bright" Reactive lighting (User Color)
 */

#include "Adafruit_TinyUSB.h"
#include "TLx493D_inc.hpp"
#include <Adafruit_NeoPixel.h>
#include "UserConfig.h"

// --- CONSTANTS ---
#define PIN_NEOPIXEL  4
#define PIN_SIMPLE    3

// --- LED SETUP ---
Adafruit_NeoPixel strip(NUM_ADDRESSABLE_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// --- HARDWARE GLOBALS ---
const uint8_t physPins[] = {BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN, BUTTON4_PIN};
const uint8_t physToHID[] = {13, 14, 15, 16};
bool currentButtonState[] = {false, false, false, false};
bool prevButtonState[] = {false, false, false, false};

struct MagCalibration {
  double x_neutral = 0.0, y_neutral = 0.0, z_neutral = 0.0;
  bool calibrated = false;
} magCal;

using namespace ifx::tlx493d;

TLx493D_A1B6 magCable(Wire1, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6 magSolder(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6* activeSensor = nullptr;

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
  // 1. Update Addressable Strip
  uint32_t c = strip.Color(r, g, b);
  strip.fill(c);
  strip.show();

  // 2. Update Simple LED (PWM)
  // Convert RGB to brightness (Luminance)
  int brightness = (r * 77 + g * 150 + b * 29) >> 8; 
  
  // Apply Global Brightness limit
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
  // MODE 0: STATIC (Solid Color)
  if (LED_MODE == 0) {
    updateHardwareLeds(LED_COLOR_R, LED_COLOR_G, LED_COLOR_B);
  }
  
  // MODE 1: BREATHING (Sine wave fade)
  else if (LED_MODE == 1) {
    float val = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
    uint8_t r = (LED_COLOR_R * (int)val) / 255;
    uint8_t g = (LED_COLOR_G * (int)val) / 255;
    uint8_t b = (LED_COLOR_B * (int)val) / 255;
    updateHardwareLeds(r, g, b);
  }
  
  // MODE 2: REACTIVE (Dim Resting -> Bright Active)
  else if (LED_MODE == 2) {
      // 1. Define Brightness Range (0-255 scaling factor)
      int minScale = 50;  // Resting dimness (approx 20%)
      int maxScale = 255; // Full brightness
      
      int currentScale = minScale;

      // 2. If moving, increase brightness
      if (totalMove > CONFIG_DEADZONE) {
         // Scale factor: multiply movement by 30 to ramp up brightness quickly
         int addedIntensity = (int)(totalMove * 30.0);
         currentScale = constrain(minScale + addedIntensity, minScale, maxScale);
      }
      
      // 3. Apply brightness scale to USER COLOR
      // This preserves the Hue/Saturation but changes Intensity
      uint8_t r = (LED_COLOR_R * currentScale) / 255;
      uint8_t g = (LED_COLOR_G * currentScale) / 255;
      uint8_t b = (LED_COLOR_B * currentScale) / 255;
      
      updateHardwareLeds(r, g, b);
  }
}

void setup() {
  // Init Hardware
  pinMode(PIN_SIMPLE, OUTPUT);
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  
  // Boot Status (Red)
  updateHardwareLeds(255, 0, 0);

#if DEBUG_MODE
  Serial.begin(115200);
#endif

  // Buttons
  for(uint8_t i = 0; i < 4; i++) pinMode(physPins[i], INPUT_PULLUP);

  // USB
  TinyUSBDevice.setID(USB_VID, USB_PID);
  usb_hid.setReportDescriptor(spaceMouse_hid_report_desc, sizeof(spaceMouse_hid_report_desc));
  usb_hid.setPollInterval(2);
  usb_hid.begin();

  while(!TinyUSBDevice.mounted()) delay(100);
  
  // Probe Sensors
  pinMode(MAG_POWER_PIN, OUTPUT);
  digitalWrite(MAG_POWER_PIN, HIGH);
  delay(10); 

  // Check Cable
  Wire1.begin(); Wire1.setClock(400000);
  bool cableFound = magCable.begin();

  // Check Solder
  Wire.begin(); Wire.setClock(400000);
  bool solderFound = magSolder.begin();

  // Decide active sensor and flash indicator
  if (cableFound) {
     activeSensor = &magCable;
     // Flash Green for Cable
     updateHardwareLeds(0, 255, 0); delay(500);
  } 
  else if (solderFound) {
     activeSensor = &magSolder;
     // Flash Cyan for Solder
     updateHardwareLeds(0, 255, 255); delay(500);
  } 
  else {
     blinkError(); 
  }

  // Go to Calibration (White Pulse)
  calibrateMagnetometer();
}

void loop() {
  // Buttons
  for(uint8_t i = 0; i < 4; i++) {
    currentButtonState[i] = (digitalRead(physPins[i]) == LOW);
    if (currentButtonState[i] != prevButtonState[i]) {
      setButtonStateHID(physToHID[i], currentButtonState[i]);
      prevButtonState[i] = currentButtonState[i];
    }
  }

  if (activeSensor) {
    readAndSendMagnetometerData();
  }
  delay(2);
}

void calibrateMagnetometer() {
  double sumX = 0, sumY = 0, sumZ = 0;
  int valid = 0;
  
  // Calibration Signal: White Pulse
  updateHardwareLeds(0, 0, 0); delay(100);
  updateHardwareLeds(255, 255, 255); 
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    double x, y, z;
    if(activeSensor->getMagneticField(&x, &y, &z)) {
      sumX += x; sumY += y; sumZ += z;
      valid++;
    }
    delay(10);
  }

  if(valid > 0) {
    magCal.x_neutral = sumX / valid;
    magCal.y_neutral = sumY / valid;
    magCal.z_neutral = sumZ / valid;
    magCal.calibrated = true;
    
    // Done: Blackout briefly before loop takes over
    updateHardwareLeds(0, 0, 0); delay(200);
  } else {
    // Retry: Red Blink
    for(int i=0; i<5; i++) {
       updateHardwareLeds(255, 0, 0); delay(50);
       updateHardwareLeds(0, 0, 0); delay(50);
    }
    calibrateMagnetometer();
  }
}

void readAndSendMagnetometerData() {
  double x, y, z;
  
  if(activeSensor->getMagneticField(&x, &y, &z)) {
      if(magCal.calibrated) {
        x -= magCal.x_neutral;
        y -= magCal.y_neutral;
        z -= magCal.z_neutral;
      }
      
      double totalMove = abs(x) + abs(y) + abs(z);
      
      // Update LEDs
      handleLeds(totalMove);

      if(abs(x) < CONFIG_DEADZONE) x = 0.0;
      if(abs(y) < CONFIG_DEADZONE) y = 0.0;
      if(abs(z) < CONFIG_ZOOM_DEADZONE) z = 0.0;

      int16_t tx = (int16_t)(-x * CONFIG_TRANS_SCALE);
      int16_t ty = (int16_t)(-y * CONFIG_TRANS_SCALE);
      int16_t tz = (int16_t)(z * CONFIG_ZOOM_SCALE);
      int16_t rx = (int16_t)(y * CONFIG_ROT_SCALE);
      int16_t ry = (int16_t)(x * CONFIG_ROT_SCALE);
      
      send_tx_rx_reports(tx, ty, tz, rx, ry, 0);
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

  uint8_t tx_report[6] = {(uint8_t)tx, (uint8_t)(tx>>8), (uint8_t)ty, (uint8_t)(ty>>8), (uint8_t)tz, (uint8_t)(tz>>8)};
  usb_hid.sendReport(1, tx_report, 6);
  
  delayMicroseconds(500);
  
  uint8_t rx_report[6] = {(uint8_t)rx, (uint8_t)(rx>>8), (uint8_t)ry, (uint8_t)(ry>>8), (uint8_t)rz, (uint8_t)(rz>>8)};
  usb_hid.sendReport(2, rx_report, 6);
}