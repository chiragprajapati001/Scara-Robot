#include <AccelStepper.h>

// Motor A (Stepper) pins
#define dirPinA 6
#define stepPinA 7
#define buttonForwardA 36
#define buttonBackwardA 37

// Motor B (Stepper) pins
#define dirPinB 4
#define stepPinB 5
#define buttonForwardB 38
#define buttonBackwardB 39

// DC Motor pins
#define IN1 25    // Direction control pin 1
#define IN2 24    // Direction control pin 2
#define buttonForwardDC 34  // Button for forward direction
#define buttonBackwardDC 35 // Button for backward direction

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
  char motor;   // 'A', 'B', or 'D' (for DC motor)
  int steps;   // Number of steps or direction (-1 for backward, 1 for forward)
};

// Define a sequence storage array
StepRecord stepSequence[1000]; // Adjust size as needed
int sequenceIndex = 0;         // Index for storing steps
bool recording = true;         // Flag for recording mode
bool replaying = false;        // Flag for continuous replay mode

void setup() {
  // Initialize stepper motors
  motorA.setMaxSpeed(6400);
  motorA.setAcceleration(800);
  motorB.setMaxSpeed(6400);
  motorB.setAcceleration(800);

  // Initialize DC motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Initialize button pins
  pinMode(buttonForwardA, INPUT_PULLUP);
  pinMode(buttonBackwardA, INPUT_PULLUP);
  pinMode(buttonForwardB, INPUT_PULLUP);
  pinMode(buttonBackwardB, INPUT_PULLUP);
  pinMode(buttonForwardDC, INPUT_PULLUP);
  pinMode(buttonBackwardDC, INPUT_PULLUP);
  pinMode(buttonReplay, INPUT_PULLUP);
  pinMode(buttonEraseSequence, INPUT_PULLUP);

  pinMode(LED, OUTPUT);

  Serial.begin(9600);
  Serial.println("Dual Motor and DC Motor Control Ready.");
}

void loop() {
  // Motor A (Stepper) control
  if (!replaying && digitalRead(buttonForwardA) == LOW) {
    performStep('A', 1); // Forward 1 step for Motor A
    delay(1);          // Debounce delay
  }
  if (!replaying && digitalRead(buttonBackwardA) == LOW) {
    performStep('A', -1); // Backward 1 step for Motor A
    delay(1);           // Debounce delay
  }

  // Motor B (Stepper) control
  if (!replaying && digitalRead(buttonForwardB) == LOW) {
    performStep('B', 1); // Forward 1 step for Motor B
    delay(1);          // Debounce delay
  }
  if (!replaying && digitalRead(buttonBackwardB) == LOW) {
    performStep('B', -1); // Backward 1 step for Motor B
    delay(1);           // Debounce delay
  }

  // DC Motor control
  if (!replaying && digitalRead(buttonForwardDC) == LOW) {
    performStep('D', 1); // Forward for DC Motor
    delay(1);          // Debounce delay
  }
  if (!replaying && digitalRead(buttonBackwardDC) == LOW) {
    performStep('D', -1); // Backward for DC Motor
    delay(1);           // Debounce delay
  }

  // Replay button
  if (digitalRead(buttonReplay) == LOW) {
    delay(10); // Debounce delay
    if (!replaying) {
      Serial.println("Replaying recorded steps continuously...");
      replaying = true;
    } else {
      Serial.println("Replay stopped.");
      replaying = false;
    }
    delay(500); // Debounce delay to prevent multiple triggers
  }

  // Continuous replay
  while (replaying) {
    replaySequence();
    delay(1000);
    if (digitalRead(buttonReplay) == LOW) {
      Serial.println("Replay stopped.");
      replaying = false;
      delay(500); // Debounce delay
    }
  }

  // Erase sequence button
  if (digitalRead(buttonEraseSequence) == LOW) {
    eraseSequence();
    delay(100); // Debounce delay
  }
}

// Perform step or DC motor action
void performStep(char motor, int steps) {
  if (motor == 'A' || motor == 'B') {
    AccelStepper* selectedMotor = (motor == 'A') ? &motorA : &motorB;
    selectedMotor->move(steps);
    while (selectedMotor->distanceToGo() != 0) {
      selectedMotor->run();
    }
  } else if (motor == 'D') { // DC Motor
    if (steps > 0) { // Forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else { // Backward
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
    delay(100); // Simulate a motor run duration
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
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
  Serial.println(" steps/direction.");
}

// Replay the recorded sequence
void replaySequence() {
  for (int i = 0; i < sequenceIndex; i++) {
    performStep(stepSequence[i].motor, stepSequence[i].steps);
  }
  Serial.println("Replay cycle complete.");
}

// Erase the recorded sequence
void eraseSequence() {
  sequenceIndex = 0; // Reset the sequence index
  flash_LED(3);
  Serial.println("Step sequence erased.");
}

// Flash LED for feedback
void flash_LED(int t) {
  for (int i = 0; i < t; i++) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}
