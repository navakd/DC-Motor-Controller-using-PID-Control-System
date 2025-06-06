#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Keypad.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Create an OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define the keypad size and the keypad matrix
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

// Define the keypad pins
byte rowPins[ROWS] = {4, 5, 6, 7}; // Connect the row pins to these Arduino pins
byte colPins[COLS] = {8, 9, 10};   // Connect the column pins to these Arduino pins

// Define motor control pins
const int ENA = 11;  // Enable A pin of the L298N motor driver
const int IN1 = 12; // IN1 pin of the L298N motor driver
const int IN2 = 13; // IN2 pin of the L298N motor driver

// Encoder pins
const int ENCODER_A = 3; // Encoder A pin
const int ENCODER_B = 2; // Encoder B pin
volatile int encoderPos = 0;
int prevEncoderPos = 0;
unsigned long prevTime = 0;
unsigned long interval = 1000; // 1 second interval for RPM calculation

// Motor speed control variables
const int MAX_SPEED = 255; // Maximum speed (PWM value)
const int MIN_SPEED = 0;  // Minimum speed (PWM value)
int currentSpeed = 0; // Current motor speed
int setRPM = 0; // Target RPM set by keypad
bool motorRunning = false; // Motor running state
unsigned long previousMillis = 0; // Time tracking

// Create the Keypad object
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Motor specifications
const int MOTOR_MAX_RPM = 1000; // Maximum RPM at rated voltage
const int RATED_VOLTAGE = 12; // Rated voltage of the motor
const int SUPPLY_VOLTAGE = 12; // Ensure this is the actual voltage supplied to the motor driver

// Encoder specifications
const int CPR = 840; // Counts per revolution at output shaft

// PID controller variables
float kp = 1.5, ki = 0.8, kd = 0.09; // PID coefficients
float integral = 0, previousError = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    //for(;;);
  }
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Set motor control pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // Initially, stop the motor
  stopMotor();

  // Set encoder pins as input
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // Attach interrupt to encoder A pin
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
}

void loop() {
  char key = keypad.getKey(); // Read the pressed key
  
  if (key != NO_KEY) { // Check if a key is pressed
    Serial.print("Pressed: ");
    Serial.println(key); // Print the pressed key to Serial Monitor
    
    // Set motor speed based on key pressed
    if (key == '0') {
      stopMotor();
      setRPM = 0;
      motorRunning = false;
    } else if (key >= '1' && key <= '9') {
      setRPM = map(key - '0', 1, 9, 100, MOTOR_MAX_RPM); // Map key to RPM range (100 to 1000)
      motorRunning = true;
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - prevTime >= interval) {
    int deltaPos = encoderPos - prevEncoderPos;
    prevEncoderPos = encoderPos;
    prevTime = currentTime;
    
    // Calculate actual RPM
    float actualRPM = (deltaPos * 60.0) / (CPR * (interval / 1000.0));
    int errorRPM = setRPM - actualRPM;
    if (motorRunning) {
      // PID controller
      float error = setRPM - actualRPM;
      integral += error * (interval / 1000.0);
      float derivative = (error - previousError) / (interval / 1000.0);
      float output = kp * error + ki * integral + kd * derivative;
      previousError = error;

      // Adjust motor speed based on PID output
      int adjustedSpeed = map(output, 0, MOTOR_MAX_RPM, MIN_SPEED, MAX_SPEED);
      setMotorSpeed(constrain(adjustedSpeed, MIN_SPEED, MAX_SPEED));
    } else {
      stopMotor();
    }
    
    displayRPM(setRPM, actualRPM, errorRPM);
    Serial.print("Set RPM: ");
    Serial.print(setRPM);
    Serial.print(" | Actual RPM: ");
    Serial.print(actualRPM);
    Serial.print(" | Error: ");
    Serial.println(errorRPM);

    // Print values for Serial Plotter
    Serial.print(setRPM);
    Serial.print(",");
    Serial.println(actualRPM);

    // Show an indicator on OLED if RPM is being adjusted
    if (errorRPM != 0) {
      display.setCursor(0, 48); // Move to next line
      display.print("Adjusting RPM...");
      display.display();
    }
  }
  
  delay(100); // Add a small delay to debounce the keypad
}

// Function to move the motor forward
void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, currentSpeed); // Set speed
}

// Function to move the motor backward
void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, currentSpeed); // Set speed
}

// Function to stop the motor
void stopMotor() {
  currentSpeed = 0;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); // Disable motor driver (stop motor)
}

// Function to set motor speed
void setMotorSpeed(int speed) {
  currentSpeed = constrain(speed, MIN_SPEED, MAX_SPEED); // Constrain speed within range
  if (currentSpeed == 0) {
    stopMotor(); // If speed is 0, stop the motor
  } else {
    forward(); // Move the motor forward with the new speed
  }
}

// Function to calculate RPM from PWM value
int calculateRPM(int pwmValue) {
  // Calculate the voltage applied to the motor
  float voltage = (SUPPLY_VOLTAGE * pwmValue) / 255.0;
  // Calculate the RPM based on the voltage applied
  int rpm = (MOTOR_MAX_RPM * voltage) / RATED_VOLTAGE;
  return rpm;
}

// Function to display RPM on the OLED screen
void displayRPM(int setRPM, float actualRPM, int errorRPM) {
  display.clearDisplay();
  display.setTextSize(1); // Set text size to 1
  display.setTextColor(SSD1306_WHITE); // Set text color to white
  display.setCursor(0, 0); // Set cursor position
  display.print("Set RPM: ");
  display.println(setRPM); // Display the set RPM
  display.setCursor(0, 16); // Move to next line
  display.print("Actual RPM: ");
  display.println(actualRPM); // Display the actual RPM
  display.setCursor(0, 32); // Move to next line
  display.print("Error: ");
  display.println(errorRPM); // Display the error RPM
  display.display();
}

// Interrupt service routine for reading encoder pulses
void readEncoder() {
  int stateA = digitalRead(ENCODER_A);
  int stateB = digitalRead(ENCODER_B);
  if (stateA == stateB) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

