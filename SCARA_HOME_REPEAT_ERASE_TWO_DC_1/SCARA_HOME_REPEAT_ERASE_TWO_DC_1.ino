#include <AccelStepper.h>

// Motor 1 pins
#define dirPin1 4
#define stepPin1 5

// Motor 2 pins
#define dirPin2 6
#define stepPin2 7

// DC Motor Pins
#define dcMotorIn1 24 // L298N IN1
#define dcMotorIn2 25 // L298N IN2
#define dcPWMIn 3 

// Button pins for motor 1
#define buttonForward1 38
#define buttonBackward1 39

// Button pins for motor 2
#define buttonForward2 36
#define buttonBackward2 37

// DC Motor Buttons
#define dcMotorCW 34   // Clockwise button
#define dcMotorCCW 35  // Anticlockwise button

// Common buttons
#define buttonHome 40
#define buttonErase 41

// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Create stepper objects
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);

// Normal movement parameters
int maxSpeed = 3200;
int maxAccel = 3200;

// Homing movement parameters
int homeSpeed = 3200;
int homeAccel = 800;

// Step recording
#define maxSteps 500 // Reduced size for memory constraints
struct StepData {
  long step1;
  long step2;
  bool dcMotorDirection;  // True for clockwise, false for anticlockwise
  unsigned long dcMotorTime; // Time duration for DC motor action
};

StepData stepsRecorded[maxSteps];
int stepIndex = 0;
bool isRecording = true;
bool isReplaying = false;

bool dcMotorDirection = true;  // true for clockwise, false for anticlockwise

void setup() {
  // Initialize stepper motors
  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAccel);
  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAccel);

  // Initialize button pins
  pinMode(buttonForward1, INPUT_PULLUP);
  pinMode(buttonBackward1, INPUT_PULLUP);
  pinMode(buttonForward2, INPUT_PULLUP);
  pinMode(buttonBackward2, INPUT_PULLUP);
  pinMode(buttonHome, INPUT_PULLUP);
  pinMode(buttonErase, INPUT_PULLUP);

  // Initialize DC motor control pins
  pinMode(dcMotorCW, INPUT_PULLUP);
  pinMode(dcMotorCCW, INPUT_PULLUP);
  pinMode(dcMotorIn1, OUTPUT);
  pinMode(dcMotorIn2, OUTPUT);

  // Set DC motor to stopped initially
  stopDCMotor();

  Serial.begin(9600);
  Serial.println("Dual Stepper and DC Motor Control Initialized");
}

void loop() {
  // Handle stepper motor recording or replay
  if (isRecording) {
    handleStepperMovement();
  } else if (isReplaying) {
    replaySteps();
  }

  // Handle DC motor control
  handleDCMotor();

  // Check the erase button at any time
  if (digitalRead(buttonErase) == LOW) {
    eraseSequence();
  }
}

void handleStepperMovement() {
  // Check the forward and backward buttons for motor 1
  if (digitalRead(buttonForward1) == LOW) {
    stepper1.move(1);
    recordStep(1, 0, dcMotorDirection, 0); // Record step for motor 1
  } else if (digitalRead(buttonBackward1) == LOW) {
    stepper1.move(-1);
    recordStep(-1, 0, dcMotorDirection, 0); // Record step for motor 1
  }

  // Check the forward and backward buttons for motor 2
  if (digitalRead(buttonForward2) == LOW) {
    stepper2.move(1);
    recordStep(0, 1, dcMotorDirection, 0); // Record step for motor 2
  } else if (digitalRead(buttonBackward2) == LOW) {
    stepper2.move(-1);
    recordStep(0, -1, dcMotorDirection, 0); // Record step for motor 2
  }

  // Check the home button to end recording and start replay
  if (digitalRead(buttonHome) == LOW) {
    goHome();
    isRecording = false;
    isReplaying = true;
  }

  // Run the motors to execute the move commands
  stepper1.run();
  stepper2.run();
}

void handleDCMotor() {
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (digitalRead(dcMotorCW) == LOW) {
    if (dcMotorDirection != true || currentMillis - lastMillis >= 5000) {  // Rotate for 5 seconds
      rotateClockwise();
      lastMillis = currentMillis;
    }
  } else if (digitalRead(dcMotorCCW) == LOW) {
    if (dcMotorDirection != false || currentMillis - lastMillis >= 3000) {  // Rotate for 3 seconds
      rotateAnticlockwise();
      lastMillis = currentMillis;
    }
  } else {
    stopDCMotor();
  }
}

void rotateClockwise() {
  dcMotorDirection = true;  // Set direction to clockwise
  digitalWrite(dcMotorIn1, HIGH);
  digitalWrite(dcMotorIn2, LOW);
  analogWrite(dcPWMIn, 255);
  recordStep(0, 0, dcMotorDirection, millis());  // Record the DC motor action with time
}

void rotateAnticlockwise() {
  dcMotorDirection = false;  // Set direction to anticlockwise
  digitalWrite(dcMotorIn1, LOW);
  digitalWrite(dcMotorIn2, HIGH);
  analogWrite(dcPWMIn, 255);
  recordStep(0, 0, dcMotorDirection, millis());  // Record the DC motor action with time
}

void stopDCMotor() {
  digitalWrite(dcMotorIn1, LOW);
  digitalWrite(dcMotorIn2, LOW);
    analogWrite(dcPWMIn, 0);
  Serial.println("DC Motor Stopped");
}

void recordStep(long step1, long step2, bool dcMotorDirection, unsigned long dcMotorTime) {
  if (stepIndex < maxSteps) {
    stepsRecorded[stepIndex++] = {step1, step2, dcMotorDirection, dcMotorTime};
    Serial.print("Recorded Steps - Motor 1: ");
    Serial.print(step1);
    Serial.print(", Motor 2: ");
    Serial.print(step2);
    Serial.print(", DC Motor Direction: ");
    Serial.print(dcMotorDirection ? "Clockwise" : "Anticlockwise");
    Serial.print(", DC Motor Time: ");
    Serial.println(dcMotorTime);
  } else {
    Serial.println("Step Buffer Full! Cannot Record More Steps.");
  }
}

void goHome() {
  stepper1.setMaxSpeed(homeSpeed);
  stepper1.setAcceleration(homeAccel);
  stepper2.setMaxSpeed(homeSpeed);
  stepper2.setAcceleration(homeAccel);

  stepper1.moveTo(0);
  stepper2.moveTo(0);

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }

  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAccel);
  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAccel);

  Serial.println("Homing Complete for Both Motors!");
}

void replaySteps() {
  Serial.println("Replaying Steps...");
  for (int i = 0; i < stepIndex; i++) {
    if (digitalRead(buttonErase) == LOW) {
      Serial.println("Replay Interrupted by Erase Button!");
      eraseSequence();
      return;
    }

    stepper1.move(stepsRecorded[i].step1);
    stepper2.move(stepsRecorded[i].step2);

    // Implementing DC motor replay (as per the recorded direction and time)
    if (stepsRecorded[i].dcMotorDirection) {
      rotateClockwise();
    } else {
      rotateAnticlockwise();
    }

    unsigned long startMillis = millis();
    while (millis() - startMillis < stepsRecorded[i].dcMotorTime) {
      // Run motors and wait for the time duration recorded for the DC motor
      stepper1.run();
      stepper2.run();
    }

    // Stop DC motor after replay time
    stopDCMotor();

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
      stepper1.run();
      stepper2.run();
    }
  }

  Serial.println("Re-Homing After Replay...");
  goHome();
}

void eraseSequence() {
  stepIndex = 0;
  isRecording = true;
  isReplaying = false;
  Serial.println("Recorded Sequence Erased. Ready for New Recording.");
}
