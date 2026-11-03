# DEXA: Force-Controlled Adaptive Robotic Hand

DEXA is a starter firmware project for an ESP32-based robotic hand. It features force-controlled adaptive grasping, slip detection, and a built-in Wi-Fi access point hosting a responsive web dashboard for real-time control and telemetry.

## Features
* **Adaptive Grasping**: Pre-programmed grasp types (Power, Pinch, Tripod, Fragile) adjusting closure based on force feedback.
* **Slip Detection**: Automatically detects object slip using Force Sensitive Resistors (FSRs) and dynamically boosts grasping force.
* **Web Dashboard**: Self-hosted Wi-Fi Access Point and captive web UI to monitor forces and trigger grips.
* **Dual Hardware Modes**: Supports direct ESP32-to-Servo wiring (simpler) or I2C expansion via a PCA9685 driver (more scalable).

## Hardware Requirements
* **Microcontroller**: ESP32 DevKit (WROOM-32)
* **Actuators**: Up to 5 standard hobby servos (e.g., MG996R or SG90, depending on scale)
* **Sensors**: Force Sensitive Resistors (FSRs) + 10kΩ resistors for voltage dividers
* **Power Supply**: Dedicated 5V/3A (minimum) power supply for the servos. *Do not power servos directly from the ESP32 5V pin!*
* *(Optional)*: PCA9685 16-Channel 12-bit PWM Servo Driver + 470-1000µF capacitor.

## Wiring Guide

### FSR (Force Sensors) Wiring (Both Modes)
Build a voltage divider for each sensed finger (Thumb, Index, Middle):
`3.3V` --- `FSR` --- `(Node)` --- `10kΩ Resistor` --- `GND`
Connect the `(Node)` to the respective ESP32 ADC pin:
* Thumb: GPIO 32
* Index: GPIO 33
* Middle: GPIO 34
* Ring: GPIO 35 (Optional)
* Pinky: GPIO 36 (Optional)

### Actuator Wiring 
You can configure the code to run in one of two hardware modes by changing `#define USE_PCA9685` in `DEXA_Hand.ino`.

#### Option A: Direct Drive (2-Day Build Default)
*Set `#define USE_PCA9685 0`*
Servos are connected directly to the ESP32. Ring and Pinky fingers share a single servo.
* **Power**: Servo Red -> 5V/3A Supply (+), Servo Brown -> Supply GND
* **Ground**: Ensure ESP32 GND is tied to Supply GND.
* **Signal (Orange/Yellow)**:
  * Thumb: GPIO 13
  * Index: GPIO 14
  * Middle: GPIO 27
  * Ring/Pinky (Shared): GPIO 26

#### Option B: PCA9685 Expansion
*Set `#define USE_PCA9685 1`*
Uses an I2C servo driver to free up ESP32 pins.
* **Power (Servos)**: 5V/3A Supply -> PCA9685 V+ Screw Terminal.
* **Power (Logic)**: ESP32 3.3V -> PCA9685 VCC.
* **Data**:
  * ESP32 GPIO 21 (SDA) -> PCA9685 SDA
  * ESP32 GPIO 22 (SCL) -> PCA9685 SCL
* Connect servos to PCA9685 Channels 0 through 4 (Thumb, Index, Middle, Ring, Pinky).

## Software Setup
1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install the **ESP32 Board Package** in the Boards Manager.
3. Install the required libraries via the Arduino Library Manager:
   * `ESP32Servo` by Kevin Harrington (Required if using Direct Drive).
   * `Adafruit PWM Servo Driver Library` (Required if using PCA9685).
4. Open `DEXA_Hand.ino`, configure your `USE_PCA9685` setting, and click **Upload**.

## Usage

### Web Interface
1. Power on the ESP32.
2. Connect your phone or computer to the Wi-Fi Access Point:
   * **Network Name (SSID):** `DEXA`
   * **Password:** `serbot123`
3. Open a web browser and navigate to `http://192.168.4.1`.
4. Use the on-screen buttons to change grips and view real-time force data.

### Serial Commands
You can also control the hand via the Arduino Serial Monitor (Baud Rate: `115200`). Send the following single characters:
* `p` = Power Grasp
* `n` = Pinch Grasp
* `t` = Tripod Grasp
* `f` = Fragile Grasp
* `o` = Open Hand
* `s` = Stop (Freeze current position)

*Note: The Serial Plotter can be used to visualize the CSV telemetry data streaming from the ESP32.*