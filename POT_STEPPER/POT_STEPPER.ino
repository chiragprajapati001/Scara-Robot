#include <AccelStepper.h>

// Stepper motor connection pins
#define dirPin 2
#define stepPin 3

// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Define steps per revolution (e.g., 200 steps for a 1.8° step angle motor)
#define stepsPerRevolution 200

// Create an instance of AccelStepper
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

int potPin = A0; // Potentiometer connected to analog pin A0
int potValue = 0; // Variable to store potentiometer value

long targetPosition = 0; // Target position (in steps)
long currentPosition = 0; // Current position (in steps)

void setup() {
  // Set max speed and acceleration for the stepper motor
  stepper.setMaxSpeed(1000);  // Max speed in steps per second
  stepper.setAcceleration(500); // Acceleration in steps per second^2

  Serial.begin(9600);  // Initialize serial for debugging
}

void loop() {
  // delay(2000);
  // Example: Move to position 1000 steps forward, then 1000 steps backward

    // Read the potentiometer value (0-1023)
  potValue = analogRead(potPin);

  // Map the potentiometer value to a range of steps (e.g., 0 to maxSteps)
  int targetPosition = map(potValue, 0, 1023, 0, 800);

  // targetPosition = map(potValue, 0, 360, 0, 300);

  Serial.print("Goes To");
  Serial.println(targetPosition);
     stepper.moveTo(targetPosition);
      while (stepper.distanceToGo() != 0) {
    stepper.run();  // Keep running the motor towards the target
  }
  // moveToPosition(targetPosition);  // Move to target position
  // delay(2000);  // Wait for 2 seconds
}

// Function to move to a target position
void moveToPosition(long position) {
  // Set the target position
  stepper.moveTo(position);

  // While the motor hasn't reached the target position
  while (stepper.distanceToGo() != 0) {
    stepper.run();  // Keep running the motor towards the target
  }

  // Update current position after reaching the target
  currentPosition = position;
  Serial.print("Reached Position: ");
  Serial.println(currentPosition);
}
