#include <AccelStepper.h>
#include <Servo.h>

#define LED 13

// Motor Pins (Stepper, DC, Servo)
#define dirPin1 4
#define stepPin1 5
#define dirPin2 6
#define stepPin2 7
#define dcMotorPin1 24   // Direction Pin 1 for DC Motor
#define dcMotorPin2 25   // Direction Pin 2 for DC Motor
#define dcMotorPWM 3   // PWM Pin for DC Motor speed control
#define servoPin 9      // Servo Pin

// Buttons
#define buttonForward1 38
#define buttonBackward1 39
#define buttonForward2 36
#define buttonBackward2 37
#define buttonForwardDC 34
#define buttonBackwardDC 35
#define buttonHome 40
#define buttonErase 41
#define buttonServoForward 32   // Button for moving servo forward
#define buttonServoBackward 33  // Button for moving servo backward

// Motor interface type (Step, DC, Servo)
#define motorInterfaceType 1

// Create Stepper and Servo objects
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);
Servo servo;

// Movement data structure
struct Movement {
  enum ActuatorType {STEP, DC, SERVO} type;  // Type of actuator
  int value;  // Steps (stepper), Time (DC motor), or Degrees (servo)
  int direction;  // Direction (for DC motors, forward = 1, backward = -1)
};

Movement sequence[1200];  // Array to store the sequence of movements
int sequenceIndex = 0;   // Index to keep track of where to insert next movement
bool isRecording = true;
bool isReplaying = false;
int MAX_SPEED_STPR = 6400;
int MAX_ACCEL_STPR = 1200;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(buttonForward1, INPUT_PULLUP);
  pinMode(buttonBackward1, INPUT_PULLUP);
  pinMode(buttonForward2, INPUT_PULLUP);
  pinMode(buttonBackward2, INPUT_PULLUP);
  pinMode(buttonForwardDC, INPUT_PULLUP);
  pinMode(buttonBackwardDC, INPUT_PULLUP);
  pinMode(buttonHome, INPUT_PULLUP);
  pinMode(buttonErase, INPUT_PULLUP);
  pinMode(buttonServoForward, INPUT_PULLUP);
  pinMode(buttonServoBackward, INPUT_PULLUP);

  stepper1.setMaxSpeed(MAX_SPEED_STPR);
  stepper1.setAcceleration(MAX_ACCEL_STPR);
  stepper2.setMaxSpeed(MAX_SPEED_STPR);
  stepper2.setAcceleration(MAX_ACCEL_STPR);

  servo.attach(servoPin);
  Serial.begin(9600);
  Serial.println("System Initialized");
}

void loop() {
  // If we are recording movements
  if (isRecording) {

       // Check for home button to stop recording and start replay
    if (digitalRead(buttonHome) == LOW) {
      delay(100);
      isRecording = false;
      isReplaying = true;
      homeAll();  // Move everything to home position first
      replaySequence();
    }

    // Stepper motor forward/backward button press for motor 1
    if (digitalRead(buttonForward1) == LOW) {
      stepper1.move(1);  // Move stepper 1
      recordMovement(Movement::STEP, 1, 0);  // Record step for motor 1
    } else if (digitalRead(buttonBackward1) == LOW) {
      stepper1.move(-1);  // Move stepper 1
      recordMovement(Movement::STEP, -1, 0); // Record step for motor 1
    }

    // Stepper motor forward/backward button press for motor 2
    if (digitalRead(buttonForward2) == LOW) {
      stepper2.move(1);  // Move stepper 2
      recordMovement(Movement::STEP, 1, 1);  // Record step for motor 2
    } else if (digitalRead(buttonBackward2) == LOW) {
      stepper2.move(-1);  // Move stepper 2
      recordMovement(Movement::STEP, -1, 1); // Record step for motor 2
    }

    // DC motor forward/backward button press
    if (digitalRead(buttonForwardDC) == LOW) {
      digitalWrite(dcMotorPin1, HIGH);   // Set direction for forward
      digitalWrite(dcMotorPin2, LOW);    // Set direction for forward
      analogWrite(dcMotorPWM, 255);      // Forward DC motor
      recordMovement(Movement::DC, 3, 1); // Record 3 seconds forward movement
    } else if (digitalRead(buttonBackwardDC) == LOW) {
      digitalWrite(dcMotorPin1, LOW);    // Set direction for backward
      digitalWrite(dcMotorPin2, HIGH);   // Set direction for backward
      analogWrite(dcMotorPWM, 255);      // Backward DC motor
      recordMovement(Movement::DC, 3, -1); // Record 3 seconds backward movement
    }

    // Servo motor forward/backward button press
    if (digitalRead(buttonServoForward) == LOW) {
      servo.write(90);   // Move Servo to 90 degrees
      recordMovement(Movement::SERVO, 90, 1);  // Record to 90 degrees
    } else if (digitalRead(buttonServoBackward) == LOW) {
      servo.write(0);    // Move Servo to 0 degrees
      recordMovement(Movement::SERVO, 0, 1);  // Record to 0 degrees
    }

   
  }

  // If we are replaying the recorded sequence
  if (isReplaying) {
    replaySequence();
  }

  // Erase button logic
  if (digitalRead(buttonErase) == LOW) {
    eraseSequence();
  }

  // Run stepper motors
  stepper1.run();
  stepper2.run();
}

// Function to record movements
void recordMovement(Movement::ActuatorType type, int value, int direction) {
  if (sequenceIndex < 100) {
    sequence[sequenceIndex].type = type;
    sequence[sequenceIndex].value = value;
    sequence[sequenceIndex].direction = direction;
    sequenceIndex++;
  }
}

// Function to replay the sequence
void replaySequence() {
  for (int i = 0; i < sequenceIndex; i++) {
    Movement movement = sequence[i];
    switch (movement.type) {
      case Movement::STEP:
        if (movement.direction == 0) {
          stepper1.move(movement.value);  // Move stepper 1
        } else {
          stepper2.move(movement.value);  // Move stepper 2
        }
        break;
      case Movement::DC:
        if (movement.direction == 1) {
          digitalWrite(dcMotorPin1, HIGH);   // Forward DC motor
          digitalWrite(dcMotorPin2, LOW);    // Set direction for forward
        } else {
          digitalWrite(dcMotorPin1, LOW);    // Backward DC motor
          digitalWrite(dcMotorPin2, HIGH);   // Set direction for backward
        }
        analogWrite(dcMotorPWM, 255);  // Run for the specified time
        delay(movement.value * 1000);  // Move for the specified time in seconds
        analogWrite(dcMotorPWM, 0);   // Stop the motor
        break;
      case Movement::SERVO:
        servo.write(movement.value);   // Move servo to the specified angle
        delay(500);  // Allow time for servo to reach the position
        break;
    }
  }
  homeAll();  // Return all actuators to home position
}

// Function to return all actuators to home position
void homeAll() {
  stepper1.setCurrentPosition(0);  // Reset stepper 1 position
  stepper2.setCurrentPosition(0);  // Reset stepper 2 position
  servo.write(0);  // Return servo to home position
}

// Function to erase the recorded sequence
void eraseSequence() {
  sequenceIndex = 0;
  isRecording = true;
  isReplaying = false;
  Serial.println("Sequence erased, ready for new recording");
  digitalWrite(LED, HIGH);  // Blink LED to show erase
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
}
