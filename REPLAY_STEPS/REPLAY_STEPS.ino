#include <AccelStepper.h>

// Define pins for the motor
#define dirPin 6
#define stepPin 7

// Define button pins for motor control
#define buttonForward 36
#define buttonBackward 37
#define buttonReplay 40 // Button to trigger replay
#define homingSwitchPin 7 // Pin for the homing switch
#define buttonEraseSequence 41 // Button to erase the sequence
#define LED 13 // 
// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Create a stepper object
AccelStepper motor(motorInterfaceType, stepPin, dirPin);

// Step recording structure
struct StepRecord {
  int steps;   // Number of steps
};

// Define a sequence storage array
StepRecord stepSequence[1000]; // Adjust size as needed
int sequenceIndex = 0;         // Index for storing steps
bool recording = true;         // Flag for recording mode
bool replaying = false;        // Flag for continuous replay mode

void setup() {
  // Initialize motor
  motor.setMaxSpeed(1000);
  motor.setAcceleration(1000);

  // Initialize button pins
  pinMode(buttonForward, INPUT_PULLUP);
  pinMode(buttonBackward, INPUT_PULLUP);
  pinMode(buttonReplay, INPUT_PULLUP);
  pinMode(homingSwitchPin, INPUT_PULLUP); // Homing switch pin
  pinMode(buttonEraseSequence, INPUT_PULLUP); // Erase sequence button

  Serial.begin(9600);
  Serial.println("Single Motor Control Ready.");

  // Perform homing at startup
  homeStepper();
}

void loop() {
  // Check button inputs for the motor
  if (!replaying && digitalRead(buttonForward) == LOW) {
    performStep(1); // Forward 1 step
    // delay(50);      // Debounce delay
  }
  if (!replaying && digitalRead(buttonBackward) == LOW) {
    performStep(-1); // Backward 1 step
    // delay(50);       // Debounce delay
  }

  // Check replay button
  if (digitalRead(buttonReplay) == LOW) {
    delay(50); // Debounce delay
    if (!replaying) {
      Serial.println("Replaying recorded steps continuously...");
      homeStepper();  // Ensure the motor is homed before replaying
      replaying = true; // Enable continuous replay mode
    } else {
      Serial.println("Replay stopped.");
      replaying = false; // Disable replay mode
    }
    delay(500);      // Debounce delay to prevent multiple triggers
  }

  // Continuous replay
  while (replaying) {
    replaySequence();

    // Check if replay button is pressed again to stop replay
    if (digitalRead(buttonReplay) == LOW) {
      Serial.println("Replay stopped.");
      replaying = false;
      delay(500); // Debounce delay
    }
  }

   // Check if erase button is pressed during replay
    if (digitalRead(buttonEraseSequence) == LOW) {
      eraseSequence();
      replaying = false; // Stop replay mode
      delay(50);        // Debounce delay
    }
}

// Perform step and optionally record it
void performStep(int steps) {
  motor.move(steps);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }

  // Record the step if in recording mode
  if (recording) {
    if (sequenceIndex < 1000) {
      stepSequence[sequenceIndex] = {steps};
      sequenceIndex++;
    } else {
      Serial.println("Step sequence memory full!");
    }
  }

  Serial.print("Motor moved ");
  Serial.print(steps);
  Serial.println(" steps.");
}

// Replay the recorded sequence
void replaySequence() {
  homeStepper();
  delay(1000);
  recording = false; // Disable recording during replay
  for (int i = 0; i < sequenceIndex; i++) {
    int steps = stepSequence[i].steps;
    motor.move(steps);
    while (motor.distanceToGo() != 0) {
      motor.run();
    }
     // Check if replay button is pressed again to stop replay
    if (digitalRead(buttonReplay) == LOW) {
      Serial.println("Replay stopped.");
      replaying = false;
      delay(500); // Debounce delay
    }
    Serial.print("Replayed with ");
    Serial.print(steps);
    Serial.println(" steps.");
  }
  Serial.println("Replay cycle complete.");
}

// Homing function to reset the stepper to a known position
void homeStepper() {
  Serial.println("Calculating net steps for homing...");

  // Calculate net steps from the recorded sequence
  int netSteps = 0;
  for (int i = 0; i < sequenceIndex; i++) {
    netSteps += stepSequence[i].steps;
  }

  Serial.print("Net steps to return to home: ");
  Serial.println(-netSteps);

  // Move the motor back by netSteps to return to the zero position
  motor.move(-netSteps);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }

  // Reset the motor's position to 0
  motor.setCurrentPosition(0);

  Serial.println("Homing complete. Position set to 0.");
}

// Erase the recorded sequence
void eraseSequence() {

  sequenceIndex = 0; // Reset the sequence index
  flash_LED(3);
  Serial.println("Step sequence erased.");
}

void flash_LED(int t) {
  for (int i=0;i< t;i++)
  {
    digitalWrite(LED,1);
  delay(500);
  digitalWrite(LED,0);
  }
}
