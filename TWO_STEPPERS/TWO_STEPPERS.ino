#include <AccelStepper.h>

// Motor A pins
#define dirPinA 6
#define stepPinA 7
#define buttonForwardA 36
#define buttonBackwardA 37

// Motor B pins
#define dirPinB 4
#define stepPinB 5
#define buttonForwardB 38
#define buttonBackwardB 39

// Shared control pins
#define buttonReplay 40 // Button to trigger replay
#define buttonEraseSequence 41 // Button to erase the sequence
#define LED 13 // LED for feedback

// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Create stepper objects
AccelStepper motorA(motorInterfaceType, stepPinA, dirPinA);
AccelStepper motorB(motorInterfaceType, stepPinB, dirPinB);

// Step recording structure
struct StepRecord {
  char motor;   // 'A' or 'B'
  int steps;   // Number of steps
};

// Define a sequence storage array
StepRecord stepSequence[1000]; // Adjust size as needed
int sequenceIndex = 0;         // Index for storing steps
bool recording = true;         // Flag for recording mode
bool replaying = false;        // Flag for continuous replay mode

void setup() {
  // Initialize motors
  motorA.setMaxSpeed(500);
  motorA.setAcceleration(500);
  motorB.setMaxSpeed(500);
  motorB.setAcceleration(500);

  // Initialize button pins
  pinMode(buttonForwardA, INPUT_PULLUP);
  pinMode(buttonBackwardA, INPUT_PULLUP);
  pinMode(buttonForwardB, INPUT_PULLUP);
  pinMode(buttonBackwardB, INPUT_PULLUP);
  pinMode(buttonReplay, INPUT_PULLUP);
  pinMode(buttonEraseSequence, INPUT_PULLUP);

  pinMode(LED, OUTPUT);

  Serial.begin(9600);
  Serial.println("Dual Motor Control Ready.");
}

void loop() {
  // Motor A control
  if (!replaying && digitalRead(buttonForwardA) == LOW) {
    performStep('A', 1); // Forward 1 step for Motor A
    delay(1);          // Debounce delay
  }
  if (!replaying && digitalRead(buttonBackwardA) == LOW) {
    performStep('A', -1); // Backward 1 step for Motor A
    delay(1);           // Debounce delay
  }

  // Motor B control
  if (!replaying && digitalRead(buttonForwardB) == LOW) {
    performStep('B', 1); // Forward 1 step for Motor B
    delay(1);          // Debounce delay
  }
  if (!replaying && digitalRead(buttonBackwardB) == LOW) {
    performStep('B', -1); // Backward 1 step for Motor B
    delay(1);           // Debounce delay
  }

  // Check replay button
  if (digitalRead(buttonReplay) == LOW) {
    delay(10); // Debounce delay
    if (!replaying) {
      Serial.println("Replaying recorded steps continuously...");
      // homeSteppers();  // Ensure the motors are homed before replaying
      replaying = true; // Enable continuous replay mode
    } 
    else {
      Serial.println("Replay stopped.");
      replaying = false; // Disable replay mode
    }
    delay(500);      // Debounce delay to prevent multiple triggers
  }

  // Continuous replay
  while (replaying) {
    replaySequence();
    delay(1000);
    // Check if replay button is pressed again to stop replay
    if (digitalRead(buttonReplay) == LOW) {
      Serial.println("Replay stopped.");
      replaying = false;
      delay(500); // Debounce delay
    }
  }

  // Check erase button
  if (digitalRead(buttonEraseSequence) == LOW) {
    replaying = false; 
    eraseSequence();
    // Stop replay mode
    delay(100);        // Debounce delay
  }
}

// Perform step and optionally record it
void performStep(char motor, int steps) {
  AccelStepper* selectedMotor = (motor == 'A') ? &motorA : &motorB;
  selectedMotor->move(steps);
  while (selectedMotor->distanceToGo() != 0) {
    selectedMotor->run();
  }

  // Record the step if in recording mode
  if (recording) {
    if (sequenceIndex < 1000) {
      stepSequence[sequenceIndex] = {motor, steps};
      sequenceIndex++;
    } else {
      Serial.println("Step sequence memory full!");
    }
  }

  Serial.print("Motor ");
  Serial.print(motor);
  Serial.print(" moved ");
  Serial.print(steps);
  Serial.println(" steps.");
}

// Replay the recorded sequence
void replaySequence() {
  homeSteppers();
  delay(1000);
  recording = false; // Disable recording during replay
  for (int i = 0; i < sequenceIndex; i++) {
    char motor = stepSequence[i].motor;
    int steps = stepSequence[i].steps;
    performStep(motor, steps);

    // Check if replay button is pressed again to stop replay
    if (digitalRead(buttonReplay) == LOW) {
      Serial.println("Replay stopped.");
      replaying = false;
      delay(500); // Debounce delay
      break;
    }
    Serial.print("Replayed motor ");
    Serial.print(motor);
    Serial.print(" with ");
    Serial.print(steps);
    Serial.println(" steps.");
  }
  Serial.println("Replay cycle complete.");
}

// Homing function to reset both steppers to a known position
void homeSteppers() {
  Serial.println("Calculating net steps for homing...");

  int netStepsA = 0, netStepsB = 0;

  // Calculate net steps for each motor
  for (int i = 0; i < sequenceIndex; i++) {
    if (stepSequence[i].motor == 'A') {
      netStepsA += stepSequence[i].steps;
    } else if (stepSequence[i].motor == 'B') {
      netStepsB += stepSequence[i].steps;
    }
  }

  // Home Motor A
  Serial.print("Motor A net steps: ");
  Serial.println(-netStepsA);
  motorA.move(-netStepsA);
  while (motorA.distanceToGo() != 0) {
    motorA.run();
  }
  motorA.setCurrentPosition(0);

  // Home Motor B
  Serial.print("Motor B net steps: ");
  Serial.println(-netStepsB);
  motorB.move(-netStepsB);
  while (motorB.distanceToGo() != 0) {
    motorB.run();
  }
  motorB.setCurrentPosition(0);

  Serial.println("Homing complete. Both motors set to 0.");
}

// Erase the recorded sequence
void eraseSequence() {
  sequenceIndex = 0; // Reset the sequence index
  flash_LED(3);
  Serial.println("Step sequence erased.");
}

void flash_LED(int t) {
  for (int i = 0; i < t; i++) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}
