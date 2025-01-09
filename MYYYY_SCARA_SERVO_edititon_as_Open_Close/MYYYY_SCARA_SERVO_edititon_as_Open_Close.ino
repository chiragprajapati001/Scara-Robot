#include <AccelStepper.h>
#include <math.h>
#include <Servo.h>

#define LED 13

// Motor 1 pins
#define dirPin1 4
#define stepPin1 5

// Button pins for motor 1
#define buttonForward1 38
#define buttonBackward1 39

// Motor 2 pins
#define dirPin2 6
#define stepPin2 7

// Button pins for motor 2
#define buttonForward2 36
#define buttonBackward2 37

//DC Motor
#define motorEnablePin 3
#define dcMotorForwardPin 24
#define dcMotorBackwardPin 25

//Servo 
#define ServoPin 9

#define buttonServoCW  42
#define buttonServoCCW  43

// Button pins for DC motor
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
Servo Servo1;

// Normal Stepper parameters
int maxSpeed = 12800;
int maxAccel = 1600;

// Homing Stepper parameters
int homeSpeed = 3200;
int homeAccel = 400;

//Servo Parameters

int srv_del = 2; // Small delay for smooth movement
int srv_close = 180, srv_open = 0, srv_home = 90;
int srv_pos = srv_home, current_pos;
int movementThreshold = 0;

// DC parameters
int pwm = 255;
int dir;
int dc_sample = 100;
bool DC_Moved = 0;

int fduration, bduration; //DC Movements Periods
int bnet, fnet, net; //Net DC Movements Periods

// long fduration, bduration; //DC Movements Periods
// long bnet, fnet, net; //Net DC Movements Periods    //**************UnComment for More Data**********//

// Step recording Structure
#define maxSteps 500 // Reduced size for memory constraints

struct StepData {
  int step1;
  int step2;
  int DC_Dir;
  int FTime;
  int BTime;
  int Srv;
};

// // Step recording Structure                             //**************UnComment for More Data**********//
// #define maxSteps 500 // Reduced size for memory constraints
// struct StepData {
//   int step1;
//   int step2;
//   int DC_Dir;
//   long FTime;
//   long BTime;
//   int Srv;
// };


StepData stepsRecorded[maxSteps];
int stepIndex = 0;
bool isRecording = true;
bool isReplaying = false;

void setup() {
  // Initialize stepper motors
  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAccel);
  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAccel);

  //DC motors Pins Setup
  pinMode(dcMotorForwardPin,OUTPUT);
  pinMode(dcMotorBackwardPin,OUTPUT);
  pinMode(motorEnablePin,OUTPUT);

  Servo1.attach(ServoPin);

  // Initialize button pins
  pinMode(buttonForward1, INPUT_PULLUP);
  pinMode(buttonBackward1, INPUT_PULLUP); //Stepper 1

  pinMode(buttonForward2, INPUT_PULLUP);
  pinMode(buttonBackward2, INPUT_PULLUP); //Stepper 2

  pinMode(buttonForwardDC, INPUT_PULLUP);
  pinMode(buttonBackwardDC, INPUT_PULLUP); //DC Motor  

  pinMode(buttonServoCW, INPUT_PULLUP);
   pinMode(buttonServoCCW, INPUT_PULLUP);  //Servo

  pinMode(buttonHome, INPUT_PULLUP);
  pinMode(buttonErase, INPUT_PULLUP); // Replay Homing Erasing etc.

  Servo1.write(srv_home);

  Serial.begin(9600); // For Debugging
  Serial.println("Dual Stepper Motor Control Initialized");
}

void loop() {
  if (isRecording) {
    //%%%%%%%%%%%%%%%%%%%%%% STEPPER MOTORs %%%%%%%%%%%%%%%%%%%%%%%%%%
    // Check the forward and backward buttons for motor 1
    if (digitalRead(buttonForward1) == LOW) {
      stepper1.move(1);
      recordStep(1, 0, 0, 0, 0, 0); // Record step for motor 1
    } else if (digitalRead(buttonBackward1) == LOW) {
      stepper1.move(-1);
      recordStep(-1, 0, 0, 0, 0, 0); // Record step for motor 1
    }

    // Check the forward and backward buttons for motor 2
    if (digitalRead(buttonForward2) == LOW) {
      stepper2.move(1);
      recordStep(0, 1, 0, 0, 0, 0); // Record step for motor 2
    } 
    else if (digitalRead(buttonBackward2) == LOW) {
      stepper2.move(-1);
      recordStep(0, -1, 0, 0, 0, 0); // Record step for motor 2
    }
    //%%%%%%%%%%%%%%%%%%%%%% DC MOTOR %%%%%%%%%%%%%%%%%%%%%%%%%%
    // Check the forward and backward buttons for DC motor
    if (digitalRead(buttonForwardDC) == LOW) {
      dir = 1;
      fduration += 1;
      drive_DC(dir);
      delay(dc_sample);
      DC_Moved = 1;
      
      //DC Motor after cond
    if(digitalRead(buttonForwardDC) != LOW && DC_Moved == 1)
      {  
        // Serial.println(fduration);
        recordStep(0, 0, dir, fduration, 0, 0); // Record step for motor 2
        fduration = 0;
        DC_Moved = 0;
        dir = 0;
        drive_DC(dir); 
        // Serial.println("DC_F_2");
      }

    } 
    if (digitalRead(buttonBackwardDC) == LOW) {
       
      dir = -1;
      bduration += 1; 
      drive_DC(dir);
      delay(dc_sample);
      DC_Moved = 1;
      
      //DC Motor after cond
    if(digitalRead(buttonBackwardDC) != LOW && DC_Moved == 1)
      { 
      // Serial.println(bduration);
      recordStep(0, 0, dir, 0, bduration, 0); // Record step for motor 2
      bduration = 0;
      DC_Moved = 0;
      dir= 0;
      drive_DC(dir); 
        // Serial.println("DC_B_2");
      }
    } 

      //%%%%%%%%%%%%%%%%%%%%%% SERVO MOTOR %%%%%%%%%%%%%%%%%%%%%%%%%%
    if (digitalRead(buttonBackwardDC) != LOW || digitalRead(buttonForwardDC) != LOW )
    {
      drive_DC(0);
      // Serial.println("DC_0_0");
    }

    // Servo motor control
    if (digitalRead(buttonServoCW) == LOW) { // If CW button is pressed
      // if (srv_pos < max_srv_angle) {            // Ensure servo doesn't exceed 180 degrees
      //   srv_pos += 5;
        Servo1.write(srv_close);
      //   delay(srv_del); // Small delay for smooth movement
      // }
      recordStep(0,0,0,0,0,-1);
    } 
    else if (digitalRead(buttonServoCCW) == LOW) { // If CCW button is pressed
      
        Servo1.write(srv_open);
      
      recordStep(0,0,0,0,0,1);
    }
    // recordStep(0,0,0,0,0,srv_pos);

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
  else if (isReplaying) {
    replaySteps();
  }

  // Check the erase button at any time
  if (digitalRead(buttonErase) == LOW) {
    // goHome();
    eraseSequence();
  }
}




void drive_DC(int dir){
  if(dir == 1)
  {
  digitalWrite (dcMotorForwardPin,0);
  digitalWrite (dcMotorBackwardPin,1);
  analogWrite(motorEnablePin, pwm);
  // delay(time);
  }
  else if (dir == -1)
  {
  digitalWrite (dcMotorForwardPin,1);
  digitalWrite (dcMotorBackwardPin,0);
  analogWrite(motorEnablePin, pwm);
  // delay(time);
  }
  else if (dir == 0){
    analogWrite(motorEnablePin, 0);
  }

}

void recordStep(int step1, int step2, int DC_Dir, long FTime, long BTime, int srv) {
  if (stepIndex < maxSteps) {
    stepsRecorded[stepIndex++] = {step1, step2, DC_Dir, FTime, BTime, srv};
    fnet += FTime;
    bnet += BTime;
    net = fnet - bnet;//*******************Fixed
    Serial.print("Recorded- S1: ");
    Serial.print(step1);
    Serial.print(", S2: ");
    Serial.print(step2);
    Serial.print(", DC Dir: ");
    Serial.print(DC_Dir);
    Serial.print(", FTime: ");
    Serial.print(FTime);
    Serial.print(", BTime: ");
    Serial.print(BTime);
    Serial.print(", Servo Pos: ");
    Serial.println(srv);


  } else {
    Serial.println("Step Buffer Full! Cannot Record More Steps.");
  }
}

void goHome() {
    Servo1.write(srv_home); //Servo Homed

    if (net < 0)
    {
      for (int k=2; k < abs(net);k++) 
      { delay(dc_sample);
      drive_DC(1);}
    }

    else if (net > 0)
    {
      for (int k=2; k < abs(net);k++) 
       {delay(dc_sample);
        drive_DC(-1);}
    }                                        //DC Homed

  stepper1.setMaxSpeed(homeSpeed);
  stepper1.setAcceleration(homeAccel);
  stepper2.setMaxSpeed(homeSpeed);
  stepper2.setAcceleration(homeAccel);

  stepper1.moveTo(0);
  stepper2.moveTo(0);         

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }                                 //Stepper Homed

  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAccel);
  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAccel);

  Serial.println("Homing Complete for All Motors!");
}

void replaySteps() {
  Serial.println("Replaying Steps...");
  for (int i = 0; i < stepIndex; i++) {
    if (digitalRead(buttonErase) == LOW) {
      Serial.println("Replay Interrupted by Erase Button!");
      eraseSequence();
      return;
    }

    replay_servo(stepsRecorded[i].Srv);
    // delay(500);
    replay_DC(stepsRecorded[i].DC_Dir, stepsRecorded[i].FTime, stepsRecorded[i].BTime);
 
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


void replay_servo(int go_pos)
{
  if(go_pos == -1)
  {
    Servo1.write(srv_close);
  }

  else if(go_pos == 1)
  {
    Servo1.write(srv_open);
  }

  // else if(go_pos == 0)
  // {
  //   Servo1.write(srv_home);
  // }
}

void replay_DC(int dir, int fw, int bw)
{
  int maxx = fw + bw;
  // Serial.println(maxx);
  for (int j=0; j < maxx; j++)
  {
  drive_DC(dir);
  delay(dc_sample);}
}

void eraseSequence() {
  stepIndex = 0;
  isRecording = true;
  isReplaying = false;
  Serial.println("Recording Erased. Ready for New Recording.");
}