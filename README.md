# DC-Motor-Controller-using-PID-Control-System

**DC Motor Speed Controller with PID Feedback**
*Author: \Nava Kiran Deep*

---

## Overview

This project implements a DC motor speed controller using PID control on an Arduino Uno. It reads a user‐defined target RPM via a 4×3 keypad, measures actual RPM with a quadrature encoder, and adjusts motor duty cycle through an L298N driver to minimize error. 
An SSD1306 OLED displays set RPM, actual RPM, and error in real time.

---

## Features

* **Keypad Input:** 4×3 membrane keypad allows selecting one of nine discrete RPM values (mapped from 100 to 1000 RPM). Pressing “0” immediately stops the motor.
* **PID Regulation:** PID loop (tunable *Kp*, *Ki*, *Kd*) adjusts PWM to hold the motor at the desired speed.
* **Encoder Feedback:** Quadrature encoder (CPR = 840) mounted on the motor shaft measures actual RPM every second.
* **OLED Display:** 128×64 SSD1306 shows Set RPM, Actual RPM, and Error. When adjusting, a status message appears (“Adjusting RPM…”).
* **Serial Monitoring:** Prints set RPM, actual RPM, error, and CSV‐formatted data for serial plotting or logging.

---

## Hardware Components

1. **Arduino Uno** (or compatible)
2. **L298N Dual H‐Bridge Motor Driver**
3. **DC Motor with Quadrature Encoder** (e.g., 840 CPR)
4. **4×3 Keypad** (membrane style)
5. **SSD1306 OLED Display** (I²C, 128×64)
6. **Jumper Wires**
7. **External 12 V Power Supply** (for motor & L298N)
8. **Breadboard** 
---

## Wiring & Pin Assignments

### 1. Arduino ⇄ SSD1306 OLED

* **OLED VCC** → Arduino 5 V
* **OLED GND** → Arduino GND
* **OLED SCL** → Arduino A5 (SCL)
* **OLED SDA** → Arduino A4 (SDA)

 `display.begin(SSD1306_SWITCHCAPVCC, 0x3C)` for I²C address 0x3C.)*

### 2. Arduino ⇄ 4×3 Keypad

* **Keypad Rows (R1–R4)** → Arduino Pins 4, 5, 6, 7
* **Keypad Cols (C1–C3)** → Arduino Pins 8, 9, 10



### 3. Arduino ⇄ Encoder

* **Encoder A (Channel A)** → Arduino Pin 3 (Interrupt)
* **Encoder B (Channel B)** → Arduino Pin 2

*→ Attach an interrupt to pin 3 (`attachInterrupt(digitalPinToInterrupt(ENCODER_A), …)`).*

### 4. Arduino ⇄ L298N Motor Driver

* **ENA (PWM enable)** → Arduino Pin 11 (PWM)
* **IN1** → Arduino Pin 12
* **IN2** → Arduino Pin 13
* **L298N 12 V+** → External 12 V Supply
* **L298N GND** → Arduino GND + Power Supply GND (common ground)
* **Motor + / Motor –** → DC Motor terminals

*(ENA uses `analogWrite(ENA, value)` for speed control; IN1/IN2 set direction.)*

---

## Software Requirements

1. **Arduino IDE** (v1.8.19 or later)

2. Install the following Arduino libraries (via Library Manager or download):

   * **Adafruit\_GFX** – core graphics library
   * **Adafruit\_SSD1306** – SSD1306 OLED driver
   * **Keypad** – for matrix keypad scanning


## Installation & Setup

1. **Open Sketch**

   * Launch Arduino IDE.
   * Choose **File → Open...** and select `DC_Motor_PID_Controller.ino`.

2. **Install Libraries**
   In the Arduino IDE, navigate to **Sketch → Include Library → Manage Libraries…** and search/install:

   * “Adafruit GFX”
   * “Adafruit SSD1306”
   * “Keypad”

4. **Configure Board & Port**

   * **Tools → Board → “Arduino Uno”** (or your board).
   * **Tools → Port → COMx** (the port where your board is connected).

5. **Upload Code**
   Click the **Upload** button. Ensure no errors. The OLED should briefly show its splash sequence.

---

## How It Works

1. **Keypad Input**

   * The `Keypad` object scans rows/columns for a pressed key.
   * When a key `'1'` through `'9'` is detected:

     ```cpp
     setRPM = map(key - '0', 1, 9, 100, MOTOR_MAX_RPM);
     motorRunning = true;
     ```

     This maps numeric keys 1–9 to a target RPM range of 100–1000 RPM.
   * Pressing `'0'` immediately calls `stopMotor()`, sets `setRPM = 0`, and halts the PID.

2. **Encoder RPM Measurement**

   * An interrupt attached to **ENCODER\_A** (pin 3) calls `readEncoder()` on every change.
   * Inside `readEncoder()`, we compare the states of A and B to decide `encoderPos++` or `encoderPos--`.
   * Every second (`interval = 1000 ms`), we compute

     ```cpp
     float actualRPM = (deltaPos * 60.0) / (CPR * (interval / 1000.0));
     ```

     where `deltaPos = encoderPos – prevEncoderPos`.
     Since CPR = 840 counts per revolution, this yields the motor’s shaft RPM.

3. **PID Control Loop**

   * **Error** = `setRPM – actualRPM`
   * **Integral** accumulates error over time:

     ```cpp
     integral += error * (Δt);
     ```
   * **Derivative** = `(error – previousError) / Δt`
   * **Output** = `Kp * error + Ki * integral + Kd * derivative`
   * We map this `output` to a PWM range (0–255):

     ```cpp
     int adjustedSpeed = map(output, 0, MOTOR_MAX_RPM, MIN_SPEED, MAX_SPEED);
     setMotorSpeed(constrain(adjustedSpeed, MIN_SPEED, MAX_SPEED));
     ```
   * `setMotorSpeed()` simply stores `currentSpeed` and calls `forward()` (driving IN1 = HIGH, IN2 = LOW, `analogWrite(ENA, currentSpeed)`).

4. **OLED Display**

   * Each second, `displayRPM(setRPM, actualRPM, errorRPM)` is invoked. It:

     1. Clears the buffer
     2. Writes:

        ```
        Set RPM:     <setRPM>
        Actual RPM:  <actualRPM>
        Error:       <errorRPM>
        ```
     3. If `errorRPM != 0`, also prints “Adjusting RPM…” at line 48px.
     4. Finally calls `display.display()` to update the 128×64 screen.

5. **Serial Output**

   * Useful for debugging or plotting in the Serial Plotter. Each second prints:

     ```
     Set RPM: <setRPM> | Actual RPM: <actualRPM> | Error: <error>
     ```
   * And CSV‐style:

     ```
     <setRPM>,<actualRPM>
     ```
   * Baud rate is set to 9600 (`Serial.begin(9600)`).

---

## Wiring Diagram 

![image](https://github.com/user-attachments/assets/837fb04e-9731-4c54-961f-0c777277a3cc)


## Parameter Tuning (PID Constants)

Default PID coefficients in the code:

```cpp
float kp = 1.5, ki = 0.8, kd = 0.09;
```

* **Kp (Proportional):** High Kp reduces steady‐state error but may cause oscillations.
* **Ki (Integral):** Eliminates steady‐state offset but can induce overshoot and windup if too large.
* **Kd (Derivative):** Adds damping, reduces overshoot, and helps stabilize transient response.

### How to Tune

1. **Start with all terms = 0**:

   ```
   kp = 0; ki = 0; kd = 0;
   ```

   Motor will not respond to RPM changes.
2. **Increase Kp** until the motor approaches target with small oscillations.
3. **Add Ki** slowly to eliminate any steady‐state offset (motor settles exactly at set RPM).
4. **Add Kd** to dampen overshoot if needed.
5. Observe response on OLED and Serial Plotter (use Arduino IDE’s Serial Plotter: **Tools → Serial Plotter**; set baud 9600).

---

## Usage Instructions

1. **Power Up**

   * Connect the 12 V power supply to L298N.
   * Connect Arduino to PC via USB (power & serial).

2. **Set Desired RPM**

   * On the keypad, press any key `1`–`9`.
   * The code maps `'1' → 100 RPM`, `'2' → 200 RPM`, …, `'9' → 1000 RPM`.
   * OLED will update “Set RPM” immediately.

3. **Observe Motor Behavior**

   * The PID loop runs every 1 second to adjust PWM.
   * The motor will accelerate or decelerate to reach the set RPM.
   * OLED shows “Actual RPM” and “Error.”
   * A status “Adjusting RPM…” appears until error = 0.

4. **Stop Motor**

   * Press `'0'` on keypad at any time.
   * Motor stops, and “Set RPM: 0” is displayed.

5. **Serial Output**

   * Open **Serial Monitor** (Ctrl+Shift+M) at 9600 baud.
   * Each second, you will see lines like:

     ```
     Set RPM: 600 | Actual RPM: 587.2 | Error: 12.8
     ```
   * Also CSV lines (`600,587.2`) allow plotting actual vs. set RPM.

---

## Code Structure & Key Functions

1. **`setup()`**

   * Initializes Serial (9600 baud).
   * Initializes OLED (`display.begin(...)`) and clears it.
   * Sets motor driver pins (`ENA`, `IN1`, `IN2`) to OUTPUT and calls `stopMotor()`.
   * Configures encoder pins (`ENCODER_A`, `ENCODER_B`) as INPUT.
   * Attaches interrupt to `ENCODER_A` for rising/falling edges → `readEncoder()`.

2. **`loop()`**

   * Reads `keypad.getKey()`. If a key is pressed:

     * If `'0'`, calls `stopMotor()`, `setRPM = 0`, `motorRunning = false`.
     * If `'1'–'9'`, computes `setRPM = map(...)` and `motorRunning = true`.
   * Every `interval` (1000 ms):

     1. Compute `deltaPos = encoderPos – prevEncoderPos`.
     2. Compute `actualRPM = (deltaPos * 60) / (CPR * 1.0)`.
     3. If `motorRunning`:

        * Compute PID terms (`error`, `integral`, `derivative`).
        * `float output = kp * error + ki * integral + kd * derivative;`
        * Map `output` to `adjustedSpeed` (0–255 PWM).
        * Call `setMotorSpeed(constrain(...))`.
          Else, call `stopMotor()`.
     4. `displayRPM(setRPM, actualRPM, error);`
     5. Print values to Serial.
     6. If `error != 0`, show “Adjusting RPM…” on OLED.
   * `delay(100)` at end for keypad debounce.

3. **`forward()`, `backward()`, `stopMotor()`, `setMotorSpeed(int)`**

   * `forward()`: `IN1=HIGH`, `IN2=LOW`, `analogWrite(ENA, currentSpeed)`.
   * `backward()`: `IN1=LOW`, `IN2=HIGH`, `analogWrite(ENA, currentSpeed)`.
   * `stopMotor()`: sets `currentSpeed=0`, `IN1=IN2=LOW`, `analogWrite(ENA, 0)`.
   * `setMotorSpeed(int speed)`: constrains `currentSpeed`, calls `stopMotor()` if 0, else calls `forward()`.

4. **`calculateRPM(int pwmValue)`** *(Unused in main loop)*

   * Computes an estimated open‐loop RPM from a PWM value and supply voltage.
   * Returns `(MOTOR_MAX_RPM * (voltage  / RATED_VOLTAGE))`.

5. **`displayRPM(int, float, int)`**

   * Clears OLED, sets text size 1, color WHITE.
   * Prints Set RPM, Actual RPM, Error on separate lines.
   * Updates display buffer.

6. **`readEncoder()`** (ISR)

   * Reads digital states of `ENCODER_A` and `ENCODER_B`.
   * If `stateA == stateB`, `encoderPos++` else `encoderPos--`.

---

## Customization & Extensibility

* **Change Target RPM Range:**
  Adjust `map(key - '0', 1, 9, 100, MOTOR_MAX_RPM)` to suit a different RPM span.
* **Adjust CPR Value:**
  If your encoder has a different counts‐per‐revolution, update `CPR = <new value>`.
* **Tune PID Gains:**
  Modify `kp`, `ki`, `kd` at the top for faster response or less overshoot.

* **Two‐Direction Control:**
  Currently, `forward()` is always called; to reverse direction based on sign of error, adapt code to call `backward()` when `error < 0`.

---

## Troubleshooting

* **OLED Doesn’t Initialize:**

  * Check I²C wiring (SDA → A4, SCL → A5).
  * Confirm address `0x3C` in `display.begin(...)`.
* **Encoder Counts Incorrect:**

  * Verify A/B wiring to digital pins 2 & 3.
  * Ensure pull‐up resistors if needed (use `pinMode(ENCODER_A, INPUT_PULLUP)`).
* **Motor Hysteresis or Stalls:**

  * Fine‐tune PID constants.
  * Check motor driver supply voltage and wiring.
  * Ensure motor can handle load (stalling reduces RPM).
* **Keypad Reads Unexpected Values:**

  * Confirm row/column mapping matches `keys[ROWS][COLS]`.
  * Debouncing handled by `delay(100)`; adjust if needed.

---

## Repository Structure

```
DC-Motor-PID-Controller/
├── DC_Motor_PID_Controller.ino   ← Main Arduino sketch
├── README.md                     ← This file
├── wiring_diagram.png           


