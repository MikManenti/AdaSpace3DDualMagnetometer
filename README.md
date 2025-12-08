# AdaSpace3D
The definitive firmware for DIY SpaceMice. Replaces glitchy mouse/keyboard emulation with Native 3DConnexion driver support. Compatible with both the original Salim Benbouzid design and modern remixes. Features smooth 5DOF control, reactive LED feedback, and auto-wiring detection.


# AdaSpace3D 🚀

**The definitive firmware upgrade for DIY SpaceMice.**

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/) ![Platform](https://img.shields.io/badge/platform-RP2040-red.svg) ![Status](https://img.shields.io/badge/status-Stable-green.svg)

Originally developed for **Leo SpaceLab's** design, **AdaSpace3D** is a drop-in firmware replacement for any DIY SpaceMouse using an **RP2040** and **TLx493D** sensor.

It fixes the biggest issue with previous DIY firmwares: **It replaces glitchy mouse/keyboard emulation with Native 3DConnexion Driver Support.**

Your DIY build will now be recognized by Windows/macOS as a genuine **SpaceMouse Pro Wireless**. No more glitchy shortcuts, no more keybinding headaches—just buttery smooth 5DOF navigation in Fusion360, Blender, SolidWorks, and more.

---

## ✨ Features

* **Native Driver Support:** Emulates the official 3DConnexion USB protocol. Works out-of-the-box with standard drivers.
* **Unified Firmware:** One file for everyone. The code **automatically detects** if your sensor is connected via **Stemma QT (Cable)** or **Soldered Headers**.
* **Reactive Lighting:**
    * **Dual Drive:** Supports both Addressable (NeoPixel) and Standard LEDs simultaneously.
    * **Smart Feedback:** LED glows dim when idle and brightens as you move the knob.
* **5DOF Navigation:** Smooth X, Y, Z translation + Pitch and Roll (Twist axis disabled by hardware physics limitations).
* **Bulletproof Flasher:** Includes a custom "One-Click" build script that handles libraries, compilers, and upload automatically.

---

## 🛠️ Hardware Support

This firmware is designed for the **Adafruit QT Py RP2040**, but works on other RP2040 boards with minor pin changes.

| Component | Pin (Default) | Notes |
| :--- | :--- | :--- |
| **Sensor** | **TLx493D** | Auto-detects on `Wire1` (Stemma) or `Wire` (Solder). |
| **Buttons** | A0, A1, A2, A3 | Mapped to HID Buttons 13, 14, 15, 16. |
| **NeoPixel** | GPIO 4 | Addressable RGB Strip (WS2812). |
| **Simple LED** | GPIO 3 | Standard 2-leg LED (PWM brightness). |

> **Note:** The firmware drives **GPIO 3 and GPIO 4 simultaneously**. You can connect your LED to either pin depending on your build, and change the behavior in `UserConfig.h`.

---

## 🚀 Quick Start (Windows)

We have included a "One-Click" tool so you don't need to install Arduino IDE or mess with libraries manually.

1.  **Download** this repository as a ZIP and extract it.
2.  Open `UserConfig.h` in any text editor to customize your settings (Sensitivity, LED Colors, etc.).
3.  **Plug in** your RP2040 device.
4.  Double-click **`FLASH.bat`**.

The script will automatically:
* Download the compiler (Arduino CLI).
* Install the RP2040 Core and Infineon Sensor libraries.
* Compile the firmware with the correct "SpaceMouse" USB ID.
* Detect your device and flash it.

---

## ⚙️ Configuration (`UserConfig.h`)

You can tweak the feel of your SpaceMouse without touching the complex code.

```cpp
// --- SENSOR SETTINGS ---
// Increase if movement feels too slow. Default: 150.0
#define CONFIG_TRANS_SCALE     150.0  

// --- LED CONFIGURATION ---
// 0 = Static (Solid Color)
// 1 = Breathing (Pulse)
// 2 = Reactive (Dim resting color, Brightens on movement)
#define LED_MODE            2

// Choose your preferred color (RGB 0-255)
#define LED_COLOR_R         0
#define LED_COLOR_G         255
#define LED_COLOR_B         255
