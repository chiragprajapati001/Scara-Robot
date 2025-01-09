#include <AccelStepper.h>

// Motor pins
#define dirPin 6   // Direction pin for the stepper motor
#define stepPin 7  // Step pin for the stepper motor

// Button pins
#define buttonForward 36   // Button to move the motor forward
#define buttonBackward 37  // Button to move the motor backward

// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Create stepper object
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);
int maxSpeed = 25600;
int maxAccel = 25600;

void setup() {
  // Initialize the stepper motor
  stepper.setMaxSpeed(maxSpeed);       // Maximum speed (steps per second)
  stepper.setAcceleration(maxAccel);  // Acceleration (steps per second^2)

  // Initialize button pins
  pinMode(buttonForward, INPUT_PULLUP); // Button for forward movement
  pinMode(buttonBackward, INPUT_PULLUP); // Button for backward movement

  Serial.begin(9600);
  Serial.println("Stepper Motor Control Initialized");
}

void loop() {
  // Check the forward button
  if (digitalRead(buttonForward) == LOW) { // Button pressed
    stepper.move(1); // Move 1 step forward
  }

  // Check the backward button
  else if (digitalRead(buttonBackward) == LOW) { // Button pressed
    stepper.move(-1); // Move 1 step backward
  }

  // Run the motor to execute the move command
  stepper.run();
}
