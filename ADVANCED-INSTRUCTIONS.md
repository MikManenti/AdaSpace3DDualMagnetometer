# AdaSpace3D - Manual Build & Flash Instructions

These instructions are for users who want to build and flash the firmware manually without using `FLASH.bat` (which does everything described below automatically).

---

## Prerequisites

1. **Download Arduino CLI** (v1.1.1 or later)  
   [arduino-cli_1.1.1_Windows_64bit.zip](https://github.com/arduino/arduino-cli/releases/download/v1.1.1/arduino-cli_1.1.1_Windows_64bit.zip)

2. **Extract** `arduino-cli.exe` somewhere accessible (e.g., `C:\arduino-cli\`)

3. **Add it to your PATH**, or use the full path in commands below

---

## One-Time Setup

Open a terminal (PowerShell or CMD) and run these commands:

```bash
# Initialize Arduino CLI config
arduino-cli config init

# Add RP2040 board support URL
arduino-cli config set board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

# Update core index
arduino-cli core update-index

# Install RP2040 core
arduino-cli core install rp2040:rp2040

# Install required libraries
arduino-cli lib install "Adafruit TinyUSB Library"
arduino-cli lib install "XENSIV 3D Magnetic Sensor TLx493D"
arduino-cli lib install "Adafruit NeoPixel"
```

---

## Building the Firmware

### Step 1: Create a Clean Sketch Folder

Create a folder named `AdaSpace3D` and copy **ONLY** these two files into it:
- `AdaSpace3D.ino`
- `UserConfig.h`

### Step 2: Compile with Custom USB Descriptors

Navigate to the **parent directory** of your `AdaSpace3D` sketch folder, then run the appropriate command for your shell:

#### CMD (Windows Command Prompt)

```cmd
arduino-cli compile --fqbn "rp2040:rp2040:adafruit_qtpy:usbstack=tinyusb" --build-property "build.vid=0x256f" --build-property "build.pid=0xc631" --build-property "build.usbvid=-DUSBD_VID=0x256f" --build-property "build.usbpid=-DUSBD_PID=0xc631" --build-property "build.usb_product=\"SpaceMouse Pro Wireless\"" --build-property "build.usb_manufacturer=\"3Dconnexion\"" --output-dir "./output" "./AdaSpace3D"
```

#### PowerShell

```powershell
arduino-cli compile --fqbn "rp2040:rp2040:adafruit_qtpy:usbstack=tinyusb" --build-property "build.vid=0x256f" --build-property "build.pid=0xc631" --build-property "build.usbvid=-DUSBD_VID=0x256f" --build-property "build.usbpid=-DUSBD_PID=0xc631" --build-property "build.usb_product=`"SpaceMouse Pro Wireless`"" --build-property "build.usb_manufacturer=`"3Dconnexion`"" --output-dir "./output" "./AdaSpace3D"
```

#### Linux / macOS

```bash
arduino-cli compile --fqbn "rp2040:rp2040:adafruit_qtpy:usbstack=tinyusb" \
  --build-property "build.vid=0x256f" \
  --build-property "build.pid=0xc631" \
  --build-property "build.usbvid=-DUSBD_VID=0x256f" \
  --build-property "build.usbpid=-DUSBD_PID=0xc631" \
  --build-property 'build.usb_product="SpaceMouse Pro Wireless"' \
  --build-property 'build.usb_manufacturer="3Dconnexion"' \
  --output-dir "./output" \
  "./AdaSpace3D"
```

This creates a `.uf2` file in the `./output` folder.

---

## Flashing the Firmware

### Step 1: Enter Bootloader Mode on Your QT Py RP2040

1. **HOLD** the BOOT button
2. **PRESS and RELEASE** the RESET button
3. **RELEASE** the BOOT button

A new drive called `RPI-RP2` will appear in your file manager.

### Step 2: Copy the Firmware

Copy the `AdaSpace3D.ino.uf2` file from the `./output` folder to the `RPI-RP2` drive.

The device will automatically restart with the new firmware installed.

---

## USB Descriptor Reference

These build properties make the device appear as a 3DConnexion SpaceMouse:

| Property | Value |
|----------|-------|
| VID (Vendor ID) | `0x256f` (3Dconnexion) |
| PID (Product ID) | `0xc631` (SpaceMouse Pro Wireless) |
| Product Name | SpaceMouse Pro Wireless |
| Manufacturer | 3Dconnexion |

> [!NOTE]
> The TinyUSB stack is required for HID functionality.

---

## Troubleshooting

### "Board not found" error
- Ensure you ran: `arduino-cli core install rp2040:rp2040`

### "Library not found" error
- Run the lib install commands from the [One-Time Setup](#one-time-setup) section

### "Permission denied" when copying UF2
- Ensure the `RPI-RP2` drive is mounted and writable
- Try running your terminal as Administrator (Windows)

### Device not recognized by 3DConnexion driver
- Verify the USB VID/PID are correctly set in the compile command
- Check Device Manager to see what VID/PID the device reports
