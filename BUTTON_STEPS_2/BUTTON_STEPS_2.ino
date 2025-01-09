#include <AccelStepper.h>

// Stepper motor connection pins
// #define dirPin 2
// #define stepPin 3

// // Button pins
// #define buttonIncreasePin 36
// #define buttonDecreasePin 37


// // Stepper motor connection pins
#define dirPin 6
#define stepPin 7

// Button pins
#define buttonIncreasePin 38
#define buttonDecreasePin 39


// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Define steps per revolution (e.g., 200 steps for a 1.8° step angle motor)
#define stepsPerRevolution 200

// Create an instance of AccelStepper
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

int potPin = A0; // Potentiometer connected to analog pin A0
int potValue = 0; // Variable to store potentiometer value

long targetPosition = 0; // Target position (in steps)
const long maxSteps = 520; // Maximum steps
const long minSteps = -520;   // Minimum steps

void setup() {
  // Set max speed and acceleration for the stepper motor
  stepper.setMaxSpeed(300);  // Max speed in steps per second
  stepper.setAcceleration(100); // Acceleration in steps per second^2

  // Initialize button pins
  pinMode(buttonIncreasePin, INPUT_PULLUP);
  pinMode(buttonDecreasePin, INPUT_PULLUP);

  Serial.begin(9600);  // Initialize serial for debugging
}

void loop() {


  // Check button inputs
  while (digitalRead(buttonIncreasePin) == LOW) {
    targetPosition += 1; // Increase position by 10 steps
    if (targetPosition > maxSteps) targetPosition = maxSteps;
    // Move to the target position
  stepper.moveTo(targetPosition);
  stepper.run();  // Keep running the motor towards the target
  
    // delay(10); // Debounce delay
  }
  
  while (digitalRead(buttonDecreasePin) == LOW) {
    targetPosition -= 1; // Decrease position by 10 steps
    if (targetPosition < minSteps) targetPosition = minSteps;
    // Move to the target position
  stepper.moveTo(targetPosition);
  stepper.run();  // Keep running the motor towards the target
  
    // delay(10); // Debounce delay
  }

  Serial.print("Target Position: ");
  Serial.println(targetPosition);

  
}
