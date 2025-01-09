#include <AccelStepper.h>

// Motor A pins
#define dirPinA 6
#define stepPinA 7
#define buttonForwardA 36
#define buttonBackwardA 37

// Motor B pins
#define dirPinB 2
#define stepPinB 3
#define buttonForwardB 38
#define buttonBackwardB 39

// DC Motor pins
#define dirPinDC1 24
#define dirPinDC2 26

#define buttonClockwiseDC 28
#define buttonAntiClockwiseDC 30

// Shared control pins
#define buttonReplay 40 // Button to trigger replay
#define buttonEraseSequence 41 // Button to erase the sequence
#define LED 13 // LED for feedback

// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Create stepper objects
AccelStepper motorA(motorInterfaceType, stepPinA, dirPinA);
AccelStepper motorB(motorInterfaceType, stepPinB, dirPinB);

// Step and DC motor recording structure
struct StepRecord {
  char motor;   // 'A', 'B', or 'D' (for DC motor)
  int duration; // Duration for DC motor or steps for stepper motors
  bool direction; // Direction for DC motor (true: clockwise, false: anticlockwise)
};

// Define a sequence storage array
StepRecord stepSequence[1000]; // Adjust size as needed
int sequenceIndex = 0;         // Index for storing steps
bool recording = true;         // Flag for recording mode
bool replaying = false;        // Flag for continuous replay mode
int deb_del = 1;
void setup() {
  // Initialize motors
  motorA.setMaxSpeed(1000);
  motorA.setAcceleration(1000);
  motorB.setMaxSpeed(1000);
  motorB.setAcceleration(1000);

  // Initialize stepper button pins
  pinMode(buttonForwardA, INPUT_PULLUP);
  pinMode(buttonBackwardA, INPUT_PULLUP);
  pinMode(buttonForwardB, INPUT_PULLUP);
  pinMode(buttonBackwardB, INPUT_PULLUP);

  // Initialize DC motor pins
  pinMode(dirPinDC1, OUTPUT);
  pinMode(dirPinDC2, OUTPUT);
  pinMode(buttonClockwiseDC, INPUT_PULLUP);
  pinMode(buttonAntiClockwiseDC, INPUT_PULLUP);

  // Initialize control pins
  pinMode(buttonReplay, INPUT_PULLUP);
  pinMode(buttonEraseSequence, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  Serial.begin(9600);
  Serial.println("Multi-Motor Control Ready.");
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

  // DC Motor control
  while (!replaying && digitalRead(buttonClockwiseDC) == LOW) {
    delay(deb_del);
    clockwiseDC();
  }
  while (!replaying && digitalRead(buttonAntiClockwiseDC) == LOW) {
    delay(deb_del);
    anticlockwiseDC();
  }
    //  digitalWrite(dirPinDC1, LOW);
    // digitalWrite(dirPinDC2, LOW);
  

  // Check replay button
  if (digitalRead(buttonReplay) == LOW) {
    delay(1); // Debounce delay
    if (!replaying) {
      Serial.println("Replaying recorded steps continuously...");
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
    eraseSequence();
    replaying = false; // Stop replay mode
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

// DC motor clockwise control

void clockwiseDC() {
  digitalWrite(dirPinDC1, HIGH);
  digitalWrite(dirPinDC2, LOW);
  
  digitalWrite(dirPinDC1, LOW);
  digitalWrite(dirPinDC2, LOW);
}

// DC motor anticlockwise control
void anticlockwiseDC() {
  digitalWrite(dirPinDC1, LOW);
  digitalWrite(dirPinDC2, HIGH);
  
  digitalWrite(dirPinDC1, LOW);
  digitalWrite(dirPinDC2, LOW);
}

// Replay the recorded sequence
void replaySequence() {
  for (int i = 0; i < sequenceIndex; i++) {
    char motor = stepSequence[i].motor;
    int duration = stepSequence[i].duration;
    bool direction = stepSequence[i].direction;

    if (motor == 'A' || motor == 'B') {
      performStep(motor, duration);
    } else if (motor == 'D') {
      digitalWrite(dirPinDC1, direction);
      digitalWrite(dirPinDC2, !direction);
      delay(duration);
      digitalWrite(dirPinDC1, LOW);
      digitalWrite(dirPinDC2, LOW);
    }

    Serial.print("Replayed motor ");
    Serial.print(motor);
    Serial.println(".");
  }
  Serial.println("Replay cycle complete.");
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
