#include <AccelStepper.h>

// Motor pins
#define dirPin 4   // Direction pin for the stepper motor
#define stepPin 5  // Step pin for the stepper motor

// Button pins
#define buttonForward 38   // Button to move the motor forward
#define buttonBackward 39  // Button to move the motor backward
#define buttonHome 40     // Button to return the motor to the home position

// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Create stepper object
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

// Normal movement parameters
int maxSpeed = 25600;
int maxAccel = 25600;

// Homing movement parameters
int homeSpeed = 800;  // Slower speed for homing
int homeAccel = 800;  // Slower acceleration for homing

// Tracks the current position (relative to home)
long currentPosition = 0;

void setup() {
  // Initialize the stepper motor
  stepper.setMaxSpeed(maxSpeed);       // Maximum speed (steps per second)
  stepper.setAcceleration(maxAccel);  // Acceleration (steps per second^2)

  // Initialize button pins
  pinMode(buttonForward, INPUT_PULLUP);  // Button for forward movement
  pinMode(buttonBackward, INPUT_PULLUP); // Button for backward movement
  pinMode(buttonHome, INPUT_PULLUP);     // Button for homing

  Serial.begin(9600);
  Serial.println("Stepper Motor Control with Homing Initialized");
}

void loop() {
  // Check the forward button
  if (digitalRead(buttonForward) == LOW) { // Button pressed
    stepper.move(1); // Move 1 step forward
    currentPosition++; // Update the position tracker
  }

  // Check the backward button
  else if (digitalRead(buttonBackward) == LOW) { // Button pressed
    stepper.move(-1); // Move 1 step backward
    currentPosition--; // Update the position tracker
  }

  // Check the home button
  else if (digitalRead(buttonHome) == LOW) { // Home button pressed
    Serial.println("Returning to Home Position with Different Speed/Acceleration...");
    stepper.setMaxSpeed(homeSpeed);       // Set homing speed
    stepper.setAcceleration(homeAccel);  // Set homing acceleration
    stepper.moveTo(0);                   // Command the stepper to move to position 0

    // Wait until the motor reaches home position
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }

    // Reset the position tracker and restore normal parameters
    currentPosition = 0;
    stepper.setMaxSpeed(maxSpeed);       // Restore normal speed
    stepper.setAcceleration(maxAccel);  // Restore normal acceleration
    Serial.println("Homing Complete!");
  }

  // Run the motor to execute the move command
  stepper.run();
}
