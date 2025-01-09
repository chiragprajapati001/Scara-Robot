#include <AccelStepper.h>


// Motor pins
#define dirPin 4   // Direction pin for the stepper motor
#define stepPin 5  // Step pin for the stepper motor

// Button pins
#define buttonForward 38   // Button to move the motor forward
#define buttonBackward 39  // Button to move the motor backward
#define buttonHome 40      // Button to return the motor to the home position
#define buttonErase 41     // Button to erase the recorded sequence

// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Create stepper object
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

// Normal movement parameters
int maxSpeed = 12800;
int maxAccel = 12800;

// Homing movement parameters
int homeSpeed = 1600;  // Slower speed for homing
int homeAccel = 1600;  // Slower acceleration for homing

// Step recording
const int maxSteps = 1000;  // Maximum number of recorded steps
long stepsRecorded[maxSteps];
int stepIndex = 0;
bool isRecording = true;  // Tracks if recording is active
bool isReplaying = false; // Tracks if replaying is active

void setup() {
  // Initialize the stepper motor
  stepper.setMaxSpeed(maxSpeed);       // Maximum speed (steps per second)
  stepper.setAcceleration(maxAccel);  // Acceleration (steps per second^2)

  // Initialize button pins
  pinMode(buttonForward, INPUT_PULLUP);  // Button for forward movement
  pinMode(buttonBackward, INPUT_PULLUP); // Button for backward movement
  pinMode(buttonHome, INPUT_PULLUP);     // Button for homing
  pinMode(buttonErase, INPUT_PULLUP);    // Button for erasing the sequence

  Serial.begin(9600);
  Serial.println("Stepper Motor Control with Re-Homing and Erase Functionality Initialized");
}

void loop() {
  if (isRecording) {
    // Check the forward button
    if (digitalRead(buttonForward) == LOW) { // Button pressed
      stepper.move(1); // Move 1 step forward
      recordStep(1);   // Record the step
    }

    // Check the backward button
    else if (digitalRead(buttonBackward) == LOW) { // Button pressed
      stepper.move(-1); // Move 1 step backward
      recordStep(-1);   // Record the step
    }

    // Check the home button to end recording and start replay
    else if (digitalRead(buttonHome) == LOW) { // Home button pressed
      Serial.println("Returning to Home and Starting Replay...");
      goHome();            // Perform homing
      isRecording = false; // Stop recording
      isReplaying = true;  // Start replaying
    }

    // Run the motor to execute the move command
    stepper.run();
  } else if (isReplaying) {
    replaySteps(); // Replay the recorded steps
  }

  // Check the erase button at any time
  if (digitalRead(buttonErase) == LOW) { // Erase button pressed
    eraseSequence();
  }
}

void recordStep(long step) {
  if (stepIndex < maxSteps) {
    stepsRecorded[stepIndex++] = step;
    Serial.print("Recorded Step: ");
    Serial.println(step);
  } else {
    Serial.println("Step Buffer Full! Cannot Record More Steps.");
  }
}

void goHome() {
  // Set homing speed and acceleration
  stepper.setMaxSpeed(homeSpeed);
  stepper.setAcceleration(homeAccel);
  stepper.moveTo(0); // Command to go to home position (step 0)

  // Wait until the motor reaches the home position
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Restore normal speed and acceleration
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxAccel);

  Serial.println("Homing Complete!");
}

void replaySteps() {
  Serial.println("Replaying Steps...");
  for (int i = 0; i < stepIndex; i++) {
    // Check if erase button is pressed during replay
    if (digitalRead(buttonErase) == LOW) {
      Serial.println("Replay Interrupted by Erase Button!");
      eraseSequence(); // Clear the sequence
      return;
    }

    stepper.move(stepsRecorded[i]); // Move as per the recorded step
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
  }

  // After replaying, perform re-homing to reset the position
  Serial.println("Re-Homing After Replay...");
  goHome();
}

void eraseSequence() {
  // Clear the recorded steps
  stepIndex = 0;
  isRecording = true;  // Set back to recording mode
  isReplaying = false; // Stop replaying
  Serial.println("Recorded Sequence Erased. Ready for New Recording.");
}
