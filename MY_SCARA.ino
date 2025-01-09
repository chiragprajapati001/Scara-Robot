#include <AccelStepper.h>

#define LED 13

// Motor 1 pins
#define dirPin1 4
#define stepPin1 5

// Motor 2 pins
#define dirPin2 6
#define stepPin2 7

#define motorEnablePin 3
#define dcMotorForwardPin 24
#define dcMotorBackwardPin 25

// Button pins for motor 1
#define buttonForward1 38
#define buttonBackward1 39

// Button pins for motor 2
#define buttonForward2 36
#define buttonBackward2 37

#define buttonForwardDC 34
#define buttonBackwardDC 35

// Common buttons
#define buttonHome 40
#define buttonErase 41

// Motor interface type (Driver with Step and Direction)
#define motorInterfaceType 1

// Create stepper objects
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);

// Normal movement parameters
int maxSpeed = 12800;
int maxAccel = 1600;

// Homing movement parameters
int homeSpeed = 3200;
int homeAccel = 400;
int dir, duration;
int last, initial = 0;
long currentTime, endTime = 0;

// Step recording
#define maxSteps 500 // Reduced size for memory constraints
struct StepData {
  int step1;
  int step2;
  int DC_dir;
  long Tyme;
};

StepData stepsRecorded[maxSteps];
int stepIndex = 0;
bool DC_moved = 0;
bool isRecording = true;
bool isReplaying = false;

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
  pinMode(buttonForwardDC, INPUT_PULLUP);
  pinMode(buttonBackwardDC, INPUT_PULLUP);
  pinMode(buttonHome, INPUT_PULLUP);
  pinMode(buttonErase, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Dual Stepper Motor Control Initialized");
  analogWrite(motorEnablePin, 0);
}

void loop() {
  if (isRecording) {
    // Check the forward and backward buttons for motor 1
    if (digitalRead(buttonForward1) == LOW) {
      stepper1.move(1);
      recordStep(1, 0, 0, 0); // Record step for motor 1
    } else if (digitalRead(buttonBackward1) == LOW) {
      stepper1.move(-1);
      recordStep(-1, 0, 0, 0); // Record step for motor 1
    }

    // Check the forward and backward buttons for motor 2
    if (digitalRead(buttonForward2) == LOW) {
      stepper2.move(1);
      recordStep(0, 1, 0, 0); // Record step for motor 2
    } 
    else if (digitalRead(buttonBackward2) == LOW) {
      stepper2.move(-1);
      recordStep(0, -1, 0, 0); // Record step for motor 2
    }

    // Check the forward and backward buttons for DC motor
    if (digitalRead(buttonForwardDC) == LOW) {
      DC_moved = 0;
      int dir = 1;
      int initial = millis();
      digitalWrite (dcMotorForwardPin,1);
      digitalWrite (dcMotorBackwardPin,0);
      analogWrite (motorEnablePin, 255);
      DC_moved = 1;
      drive_DC(dir);
    } 
    if (digitalRead(buttonBackwardDC) == LOW) {
      DC_moved = 0;
      dir = 2;
      initial = millis();
      digitalWrite (dcMotorForwardPin,1);
      digitalWrite (dcMotorBackwardPin,0);
      analogWrite(motorEnablePin, 255);
      DC_moved = 1;
      drive_DC(dir);
    } 
    //DC Motor after cond
    // if(digitalRead(buttonForwardDC) == HIGH && DC_moved == 1 || digitalRead(buttonBackwardDC) == HIGH && DC_moved == 1)
     if (!DC_moved) { 
        last = millis();
        duration = initial - last;
        DC_moved = 0;
        recordStep(0, 0, dir, duration);
      }
    //  // Record step for motor 
          


    // Check the home button to end recording and start replay
    if (digitalRead(buttonHome) == LOW) {
      goHome();
      isRecording = false;
      isReplaying = true;
    }

    // Run the motors to execute the move commands
    stepper1.run();
    stepper2.run();
  } else if (isReplaying) {
    replaySteps();
  }

  // Check the erase button at any time
  if (digitalRead(buttonErase) == LOW) {
    eraseSequence();
  }
}




void drive_DC(int dir){
  for (int i=0; i < dir;i++)
  {
    digitalWrite(LED,HIGH);
    delay(1000);
    digitalWrite(LED,LOW);
  }
}

void recordStep(long step1, long step2, long DC_Dir, long Tyme) {
  if (stepIndex < maxSteps) {
    stepsRecorded[stepIndex++] = {step1, step2, DC_Dir, Tyme};
    Serial.print("Recorded Steps - Motor 1: ");
    Serial.print(step1);
    Serial.print(", Motor 2: ");
    Serial.println(step2);
    Serial.print(", DC Motor: ");
    Serial.println(DC_Dir);
    Serial.print(", Time: ");
    Serial.println(Tyme);

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