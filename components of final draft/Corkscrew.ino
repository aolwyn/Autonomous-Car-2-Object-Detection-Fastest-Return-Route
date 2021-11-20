/**
 * Corkscrew as per the flowchart.
 * Stop if the floor marker is reached.
 * Drive around until then.
 * 
 * Used code from Bug2.ino
 * 
 * Designed by Ben, modified and developed by Adam + Ben
 */

#define REncoderPin 2
#define LEncoderPin 3
#define LIRPin      A0
#define BIRPin      A1
#define EchoPin     A2
#define TrigPin     A3
#define RIRPin      A5
/*#define REncoderPin 2
#define LEncoderPin 3
#define LIRPin      A0
#define BIRPin      A4
#define EchoPin     A1
#define TrigPin     A2
#define RIRPin      A5*/

#define TicksPerRot 80
#define MaxSpeed    255
#define MinSpeed    0
#define TargetDist  200 // 2 m = 200 cm
#define NumReadings 10
#define NumSensors  4


#include <AFMotor.h>

AF_DCMotor left_motor(1, MOTOR34_1KHZ);
AF_DCMotor right_motor(3, MOTOR12_1KHZ);


int LeftEncoderCount, LeftEncoderSmall, RightEncoderCount, RightEncoderSmall;
int distance, emitterVal;
int baseSpeed = 220;
int addedSpeed = -50;  // Determined from closed-loop testing

int readings[NumSensors][NumReadings];  // Matrix and arrays for simplicity
int totals[NumSensors], averages[NumSensors];
int readIndex = 0;

unsigned long duration, lastTimeMovement, lastTimeSensors;
bool turning = false;
double circumference = 10; //40.84; // Measured

int RightEncoderPath, RightEncoderObj;
int MaxEncoder = 0;
bool firstPass = true;

void setup() {
  // Initialize the wheel encoder interrupts.
  attachInterrupt(1, countLEncoder, RISING);
  attachInterrupt(0, countREncoder, RISING);
  interrupts();

  // Initialize all pins.
  pinMode(LIRPin, INPUT);
  pinMode(BIRPin, INPUT);
  pinMode(EchoPin, INPUT);
  pinMode(TrigPin, OUTPUT);
  pinMode(RIRPin, INPUT);

  // Initialize the serial monitor.
  Serial.begin(115200);
  Serial.println("Bug 2 Test!\n");

  // Initialize all sensor data.
  for(int thisSensor = 0; thisSensor < NumSensors; thisSensor++) {
    for(int thisReading = 0; thisReading < NumReadings; thisReading++) {
      readings[thisSensor][thisReading] = 0;
    }
    totals[thisSensor] = 0;
    averages[thisSensor] = 0;
  }

  // Initialize wheels.
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  resetEncoders();
  
  // Run a self-diagnostic on all sensors.
  selfTest();

  // Synchronize timing.
  lastTimeMovement = millis();
  lastTimeSensors = millis();
}

void loop() {
  corkscrew();
}

void corkscrew() {
  Serial.println("Reached end of path but no marker detected");
  Serial.println("Beginning corkscrew search");

  int corkDist = 50;
  int pointing = 0;
  resetEncoders();

  while(analogRead(BIRPin) < 200) {
    // Drive straight.
    if(millis() - lastTimeMovement > 500) {
      right_motor.run(FORWARD);
      left_motor.run(FORWARD);
      right_motor.setSpeed(baseSpeed);
      left_motor.setSpeed(baseSpeed + addedSpeed);
  
      // Adjust power to left motor to drive straight.
      int delta = RightEncoderSmall - LeftEncoderSmall;
      addedSpeed += delta / 2;
      if(baseSpeed + addedSpeed > MaxSpeed) left_motor.setSpeed(MaxSpeed);
      else if(baseSpeed + addedSpeed < MinSpeed) left_motor.setSpeed(MinSpeed);
      else left_motor.setSpeed(baseSpeed + addedSpeed);
    
      LeftEncoderSmall = 0;
      RightEncoderSmall = 0;
      lastTimeMovement = millis();
    }

    // Rotate and increase square radius.
    if(RightEncoderCount > corkDist) {
      corkDist += 0;
      resetEncoders();
      rotateRight();
      pointing++;
      if(pointing > 3) pointing = 0;
      resetEncoders();
    }
  }

  Serial.println("Floor marker detected");

  if(firstPass) {
    // Rotate to point away from home.
    switch(pointing) {
      case 2:
        rotateLeft();
      case 1:
        rotateLeft();
        break;
      case 3:
        rotateRight();
        break;
    }
  } else {
    // Reached home. Stop.
    stopMoving();
  }
}

void selfTest() {
  bool LWheel = true;
  bool RWheel = true;
  bool LIR = true;
  bool RIR = true;
  bool US = true;
  bool BIR = true;

  while(LWheel or RWheel or LIR or RIR or US or BIR) {
    if(LWheel and LeftEncoderSmall > 10) {
      LWheel = false;
      Serial.println("Left Wheel Turned");
    }
    if(RWheel and RightEncoderSmall > 10) {
      RWheel = false;
      Serial.println("Right Wheel Turned");
    }
    if(LIR and analogRead(LIRPin) < 50) {
      LIR = false;
      Serial.println("Left IR sensor activated");
    }
    if(RIR and analogRead(RIRPin) < 50) {
      RIR = false;
      Serial.println("Right IR sensor activated");
    }
    if(US and readUS() < 10) {
      US = false;
      Serial.println("Ultra-sonic sensor activated");
    }
    if(BIR and analogRead(BIRPin) > 50) { // 300
      BIR = false;
      Serial.println("Floor sensor activated");
    }
  }

  resetEncoders();
  Serial.println("All sensors functional. Beginning journey...\n");
  delay(2000);  // A short delay to allow for the floor marker or the obstacle to be removed.
  Serial.println("Driving straight forward\n");
}

void countREncoder(){ // Interrupt function for right encoder.
  RightEncoderCount++;
  RightEncoderSmall++;
}

void countLEncoder(){ // Interrupt function for left encoder.
  LeftEncoderCount++;
  LeftEncoderSmall++;
}

void resetEncoders() {
  LeftEncoderCount = 0;
  LeftEncoderSmall = 0;
  RightEncoderCount = 0;
  RightEncoderSmall = 0;
}

void rotateLeft() {
  delay(1000);
  Serial.println("Turning left");
  
  right_motor.run(FORWARD);
  left_motor.run(BACKWARD);
  
  while(RightEncoderCount < 1 * TicksPerRot / 3) {
    right_motor.setSpeed(200);
    left_motor.setSpeed(200);
  }

  right_motor.run(RELEASE);
  left_motor.run(RELEASE);
  delay(1000);
}

void rotateRight() {
  delay(1000);
  Serial.println("Turning right");
  
  right_motor.run(BACKWARD);
  left_motor.run(FORWARD);
  resetEncoders();

  while(RightEncoderCount < 1 * TicksPerRot / 4.0) {
    right_motor.setSpeed(200);
    left_motor.setSpeed(200);
  }

  right_motor.run(RELEASE);
  left_motor.run(RELEASE);
  delay(1000);
  resetEncoders();
}

int readUS() {
  // Read sensor data.
  // 10 microsecond pulse.
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  // Measure pulse.
  duration = pulseIn(EchoPin, HIGH);

  // Calculate and return distance.
  return duration * 0.034 / 2;
}

void stopMoving() {
  Serial.println("Stopping...");
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

  while(true) {
    delay(10000);
  }
}
