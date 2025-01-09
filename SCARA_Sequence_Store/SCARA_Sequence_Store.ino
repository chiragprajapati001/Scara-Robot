#include <AccelStepper.h>

// Stepper motor setup
#define stepPin1 6
#define dirPin1 7
#define stepPin2 4
#define dirPin2 5

AccelStepper stepperA(1, stepPin1, dirPin1);
AccelStepper stepperB(1, stepPin2, dirPin2);

// Button pins
#define buttonForwardA 36
#define buttonBackwardA 37
#define buttonForwardB 38
#define buttonBackwardB 39
#define buttonReplay 40
#define buttonStop 41

// Movement recording
char movements[1200];
int movementIndex = 0;
bool replaying = false;

void setup() {
  Serial.begin(9600);

  // Initialize stepper motors
  stepperA.setMaxSpeed(6400);
  stepperA.setAcceleration(800);
  stepperB.setMaxSpeed(6400);
  stepperB.setAcceleration(800);

  // Initialize buttons
  pinMode(buttonForwardA, INPUT_PULLUP);
  pinMode(buttonBackwardA, INPUT_PULLUP);
  pinMode(buttonForwardB, INPUT_PULLUP);
  pinMode(buttonBackwardB, INPUT_PULLUP);
  pinMode(buttonReplay, INPUT_PULLUP);
  pinMode(buttonStop, INPUT_PULLUP);

  Serial.println("Ready!");
}

void loop() {
  // Stop replay if the stop button is pressed
  if (digitalRead(buttonStop) == LOW) {
    replaying = false;
    Serial.println("Replay stopped!");
  }

  // Replay recorded movements
  if (digitalRead(buttonReplay) == LOW && !replaying) {
    replaying = true;
    Serial.println("Replaying movements...");
    replayMovements();
  }

  // If not replaying, handle button inputs
  if (!replaying) {
    handleButtonPress(buttonForwardA, 'A', stepperA, 1);
    handleButtonPress(buttonBackwardA, 'C', stepperA, -1);
    handleButtonPress(buttonForwardB, 'B', stepperB, 1);
    handleButtonPress(buttonBackwardB, 'D', stepperB, -1);
  }

  // Run steppers
  stepperA.run();
  stepperB.run();
}

// Handle button presses and record movements
void handleButtonPress(int buttonPin, char action, AccelStepper &stepper, int stepDirection) {
  if (digitalRead(buttonPin) == LOW) {
    stepper.move(stepDirection); // Actuate stepper motor
    recordMovement(action);     // Record movement
  }
}

// Record movement to array
void recordMovement(char motorAction) {
  if (movementIndex < 1200) {
    movements[movementIndex++] = motorAction;
    Serial.print("Recorded: ");
    Serial.println(motorAction);
  }
}

// Replay recorded movements
void replayMovements() {
  for (int i = 0; i < movementIndex; i++) {
    if (!replaying) break; // Stop replay if stop button is pressed

    char action = movements[i];
    if (action == 'A') {
      stepperA.move(1);
    } else if (action == 'C') {
      stepperA.move(-1);
    } else if (action == 'B') {
      stepperB.move(1);
    } else if (action == 'D') {
      stepperB.move(-1);
    }
    // Execute each movement one at a time
    stepperA.runToPosition();
    stepperB.runToPosition();
  }

  Serial.println("Replay complete.");
  replaying = false;
}
