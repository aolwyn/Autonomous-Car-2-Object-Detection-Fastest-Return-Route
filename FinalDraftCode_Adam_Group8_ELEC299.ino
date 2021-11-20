//final submission file, Prgroammed by Adam, member of group 8 of ELEC299 for Thursday 6:30-9:30 PM EST.

//setup the pins on the arduino
#define LEncoderPin 3
#define REncoderPin 2
#define echo A2
#define trig A1
#define sensor A4
#define LSensor A0
#define RSensor A5

//define some values that won't change
#define TicksPerRot 80
#define MaxSpeed    255
#define MinSpeed    0
#define TargetDist  200 // 2 m = 200 cm
#define NumReadings 10
#define NumSensors  4

//include the PID and AFMotor libraries
#include <AFMotor.h>
#include <PID_v2.h>

AF_DCMotor left_motor(1, MOTOR34_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR12_1KHZ); // right motor to M3 on motor control board

int placeholder = 1;
int turnOne = 0;
int turnTwo = 0;
int lastTimeL = 0;
int lastTimeR = 0;
int LeftEncoderCount = 0;
int RightEncoderCount = 0;
int DelayTime = 1000;
int PrevRightEncoderCount = 0;
int PrevLeftEncoderCount = 0;
int obstacleCount = 0;
int LeftEncoderSmall, RightEncoderSmall;\
int trackWidth = 0;
int trackLength = 0;
int Length1 = 0;
int Length2 = 0;
int turnLength = 0;
int firstPass = 0;
int RPWM = 0;
int LPWM = 0;
int LSens = 1, RSens = 1, BSens = 1, DSens = 1;

//PID control variables
double EncDelta = 0;
double OutDelta = -20.0;
double EncDeltaSet = 0;
double Kp = 0.5, Ki = 0.5, Kd = 0.1; // starting PID controller settings



double circumference = 10; //40.84; // Measured


unsigned long duration, lastTimeMovement, lastTimeSensors;


PID DiffSpeed(&EncDelta, &OutDelta, &EncDeltaSet, Kp, Ki, Kd, DIRECT);


//variables for averaging functions
const int numReadings = 10;
float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average


void setup() {

  attachInterrupt(1, countLEncoder, RISING); //calls on any CHANGE on pin 3 (interrupt #1, soldered to Pin3)
  attachInterrupt(0, countREncoder, RISING); //calls on any CHANGE on pin 2 (interrupt #0, connected to header on Pin2)
  interrupts();

  Serial.begin (115200);    // set up Serial library at 115200 bps
  pinMode(LSensor, INPUT);
  pinMode(RSensor, INPUT);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(sensor, INPUT);

  //setup for PID, including the bounds and auto straightening thing
  DiffSpeed.SetMode(AUTOMATIC);
  DiffSpeed.SetOutputLimits(-150, 150);
  DiffSpeed.SetSampleTime(70);

  //Set motor direction
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

  //setup with the array for the function
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  lastTimeMovement = millis();
  lastTimeSensors = millis();

  scannerSetup();
}

int flag;
void loop() {

  unsigned long myTime = millis();
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);
  float avg = getDistance();  //variable for the ultrasonic sensor

  //while the ultrasonic sensor doesn't detect anything or there's no paper being detected, go straight.
  while (1)
  {

    //while there's no object or there's no paper, go forward, update the time
    while ( avg > 20 && (getAverage(A4) > .40 || getAverage(A4) < .30) && myTime < 24000 )
    {
      flag = 1;
      myTime = millis();
      avg = getDistance();
      adjustSpeed(90);

    }//end while

    //if you're greater than 24 sec (longest time to hit both obj's and find paper), and flag = 1, and you're  on the first route
    //find the paper and turn flag to 0 so it can never activate this again
    if (myTime > 24000 && flag == 1 && firstPass == 0)
    {
      corkscrew();
      flag = 0;
    }

    //in the case of an obstacle, avoid it
    if (avg < 20)
    {
      avoidObstacle();
    }//end if

    //but if you come across the paper, change firstPass to 1 (so you don't hit the previous if) then start the return to start function.
    if (getAverage(A4) < .40 || getAverage(A4) > .30)
    {
      firstPass = 1;
      returnToStart();
      break;
    }//end if

  }//end while 1

  while (1) {}

}//end void loop

void resetEncoders() {
  LeftEncoderCount = 0;
  LeftEncoderSmall = 0;
  RightEncoderCount = 0;
  RightEncoderSmall = 0;
}//end void reset encoders

//function for getting voltage
float getVoltage(int pin) {
  float voltage = 5.0 * analogRead(pin) / 1024;
  return voltage;
}

// interrupt function for left encoder
void countLEncoder() {
  if (micros() - lastTimeL > 100) {
    LeftEncoderCount++;
    LeftEncoderSmall++;
    lastTimeL = micros();
  }
}

// interrupt function for right encoder
void countREncoder() {
  if (micros() - lastTimeR > 100) {
    RightEncoderCount++;
    RightEncoderSmall++;
    lastTimeR = micros();
  }
}

//function for getting distance
float getDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(10);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float duration = pulseIn(echo, HIGH);
  float velocity = 0.0343;
  float distance = duration * velocity / (2);
  delay(60);
  return distance;
}

//function for gettig the average in general
float getAverage(int pin) {
  total = total - readings[readIndex];
  float data = getVoltage(pin);
  readings[readIndex] = data;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;


  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  average = total / numReadings;
  //Serial.println(average);
  delay(1);        // delay in between reads for stability
  return average;
}//end get average

//function for getting the average distance
float getAverageDistance() {

  total = total - readings[readIndex];
  readings[readIndex] = getDistance();
  total = total + readings[readIndex];
  readIndex = readIndex + 1;


  if (readIndex >= numReadings) {
    readIndex = 0;
  }


  average = total / numReadings;

  //Serial.println(average);
  delay(1);
  return average;
}//end get average distance

//function for adjusting the speed
int adjustSpeed(int i) {

  EncDelta = (double)(1.0 * LeftEncoderCount - 1.0 * RightEncoderCount);
  DiffSpeed.Compute(); // compute the value of OutDelta using the PID controller
  RPWM = (int)round(i - OutDelta);
  LPWM = (int)round(i + OutDelta);

  left_motor.setSpeed(LPWM);
  right_motor.setSpeed(RPWM);
  return LPWM;
}//end adjust speed

//function for the scanner setup thing
void scannerSetup() {
  while (DSens == 1  || LeftEncoderCount < 20 || RightEncoderCount  < 20) {
    //RSens == 1 || BSens == 1 || LSens == 1||
    if (RightEncoderCount > 20) {
      Serial.println("Right Encoder Working");
    }
    if (LeftEncoderCount > 20) {
      Serial.println("Left Encoder Working");
    }
    if (getAverage(LSensor) < 2) {
      Serial.println("Left Sensor Working");
      LSens = 0;
    }
    if (getAverage(RSensor) < 2) {
      Serial.println("Right Sensor Working");
      RSens = 0;
    }
    if (getAverage(sensor) > 0.30 && getAverage(sensor) < 0.40) {
      Serial.println("Bottom Sensor Working");
      BSens = 0;
    }

    if (getAverageDistance() < 50) {
      Serial.println("Distance Sensor Working");
      DSens = 0;
    }
  }

  Serial.println("Sensor Test done");
  RightEncoderCount = 0;
  LeftEncoderCount = 0;
  delay(2000);
}//end of sensor test

//function for use of determining largest width needed to turn (only used in event of case 2)
int obstacleLength( int Length1, int Length2) {

  if (Length1 > Length2)
  {
    turnLength = Length1;
    return turnLength;

  }
  else if (Length2 > Length1)
  {
    turnLength = Length2;
    return turnLength;
  }
  else {
    turnLength = Length1;
    return turnLength;
  }
}//end obstacleLength

//function to stop moving
void stopMoving() {
  Serial.println("Stopping...");
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

  while (true) {
    delay(10000);
  }//end while
}

//function for the left turn
void leftTurn() {
  right_motor.run(BACKWARD);
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  left_motor.setSpeed(0);
  while (RightEncoderCount < 40) {
    right_motor.setSpeed(200);
    Serial.println(RightEncoderCount);
  }
  right_motor.run(BACKWARD);
  left_motor.run(BACKWARD);
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  RightEncoderCount = 0;
}//end left turn

//function for the right turn
void rightTurn() {
  left_motor.run(BACKWARD);
  right_motor.run(FORWARD);
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  right_motor.setSpeed(0);
  while (LeftEncoderCount < 40) {
    left_motor.setSpeed(200);
    right_motor.setSpeed(200);
    Serial.println(LeftEncoderCount);
  }
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  LeftEncoderCount = 0;
}//end right turn

//comment for doing the bug algorithm. assumes the left hand side is the fastest method of reaching the paper.
void avoidObstacle() {
  if (getAverageDistance() < 15) {
    obstacleCount++;
    leftTurn();
    Serial.println("Left Turn");
    /*left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(1000);*/

    while (getAverage(RSensor) < 2) { //while going left, track the side
      adjustSpeed(150);

    }
    while (RightEncoderCount < 70 || LeftEncoderCount < 70) {
      adjustSpeed(150);
      Serial.println(RightEncoderCount);
      Serial.println(LeftEncoderCount);
    }
    int prevRight = RightEncoderCount;
    int prevLeft = LeftEncoderCount;

    //placeholder detects which object it's on. it is initialized to 1. these 2 statements are used in case 3 to detect
    //the largest latitutidal distance the car must travel to avoid hitting the objects when returning to the start.
    if (placeholder == 1)
    {
      //first run,
      turnOne = LeftEncoderCount;
    }

    if (placeholder == 2)
    {
      turnTwo = LeftEncoderCount;
    }
    placeholder++;

    Serial.println("Right Turn");
    rightTurn();
    /*left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(1000);*/
    while (getAverage(RSensor) < 2) { //while going forward, track the side
      adjustSpeed(150);
    }
    while (RightEncoderCount < 100 || LeftEncoderCount < 100) { //little bit of extra distance to make sure
      //the back of car doesnt clip during the turn
      adjustSpeed(150);
      Serial.println(RightEncoderCount);
      Serial.println(LeftEncoderCount);
    }
    rightTurn();
    //delay(1000);
    while (LeftEncoderCount < prevLeft || RightEncoderCount < prevRight) {
      adjustSpeed(200);
      Serial.println(RightEncoderCount);
      Serial.println(LeftEncoderCount);
    }
    leftTurn();
    /*left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(1000);*/
  }

} //end avoid obstacle

void avoidObstacleReturn() { //same function as avoidObstacle but in reverse; (left turn becomes right, etc.)
  //function used for case 2 returning to start position.
  if (getAverageDistance() < 15) {
    rightTurn();
    Serial.println("Right Turn");
    /*left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(1000);*/
    while (getAverage(LSensor) < 2) {
      adjustSpeed(150);
    }
    while (RightEncoderCount < 70 || LeftEncoderCount < 70) {
      adjustSpeed(150);
      Serial.println(RightEncoderCount);
      Serial.println(LeftEncoderCount);
    }
    int prevRight = RightEncoderCount;
    int prevLeft = LeftEncoderCount;
    Serial.println("Left Turn");
    leftTurn();
    /*left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(1000);*/
    while (getAverage(LSensor) < 2) {
      adjustSpeed(150);
    }
    while (RightEncoderCount < 100 || LeftEncoderCount < 100) {
      adjustSpeed(150);
      Serial.println(RightEncoderCount);
      Serial.println(LeftEncoderCount);
    }
    leftTurn();
    //delay(1000);
    while (LeftEncoderCount < prevLeft || RightEncoderCount < prevRight) {
      adjustSpeed(200);
      Serial.println(RightEncoderCount);
      Serial.println(LeftEncoderCount);
    }
    rightTurn();
    /*left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(1000);*/
  }
  Serial.println("Done");
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);
} //end avoid obstacle return


//corkscrew function for finding object if marker isn't found.
void corkscrew() {
  Serial.println("Reached end of path but no marker detected");
  Serial.println("Beginning corkscrew search");

  int corkDist = 50;
  int pointing = 0;
  resetEncoders();

  while (getAverage(A4) > .40 || getAverage(A4) < .30) {
    // Drive straight while nothing has been detected (paper not detected yet)
    if (millis() - lastTimeMovement > 500) {
      adjustSpeed(150);
      lastTimeMovement = millis();
    }

    // Rotate and increase square radius.
    if (RightEncoderCount > corkDist) {
      corkDist += 0;
      resetEncoders();
      rightTurn();
      pointing++;
      if (pointing > 3) pointing = 0;
      resetEncoders();
    }
  }

  Serial.println("Floor marker detected");
  //first pass is used to represent the first time finding the marker.
  if (firstPass == 0) {
    // Rotate to point away from home.
    switch (pointing) {
      case 2:
        leftTurn();
      case 1:
        leftTurn();
        break;
      case 3:
        rightTurn();
        break;
    }
  } else {
    // Reached home. Stop.
    stopMoving();
  }//close else
}//close void corkscrew

void returnToStart()
{

  if (obstacleCount == 0) //case 1: 2 obstacles on course but doesn't detect either
  {
    leftTurn();
    leftTurn();

    while (getAverage(A4) > .40 || getAverage(A4) < .30)
    {
      adjustSpeed(100);
    }//end while

    while (getAverage(A4) < .40 || getAverage(A4) > .30)
    {
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      left_motor.run(BACKWARD);
      right_motor.run(BACKWARD);
      delay(10000);
    }//end while
  }//end if obstacle count == 0

  if (obstacleCount == 1) //case 2: 2 obstacles on course, detects 1 obstacle
  {
    left_motor.run(BACKWARD);
    right_motor.run(BACKWARD);
    float avg = getDistance();
    if (avg > 17) {
      while (avg > 17) {
        avg = getDistance();
        adjustSpeed(90);
      }//end while (avg > 17)
    }//end if (avg >17)

    avoidObstacleReturn();
    while (getAverage(A4) > .40 || getAverage(A4) < .30)
    {
      adjustSpeed(100);
    }//end while
    while (getAverage(A4) < .40 || getAverage(A4) > .30)
    {
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      left_motor.run(BACKWARD);
      right_motor.run(BACKWARD);
      delay(10000);
    }//end while at marker
  }//end if obstacle count == 1


  if (obstacleCount == 2) // CASE 3: 2 obstacles, detects both
  {
    leftTurn();

    obstacleLength(turnOne, turnTwo);

    LeftEncoderCount = 0;
    RightEncoderCount = 0;
    while (LeftEncoderCount < turnLength || RightEncoderCount < turnLength)
    {
      adjustSpeed(150);
    }
    leftTurn();
    LeftEncoderCount = 0;
    RightEncoderCount = 0;
    while (LeftEncoderCount < trackLength || RightEncoderCount < trackLength)
    {
      adjustSpeed(150);
    }

    leftTurn();
    LeftEncoderCount = 0;
    RightEncoderCount = 0;

    while (getAverage(A4) > .40 || getAverage(A4) < .30)
    {
      adjustSpeed(100);
    }//end while
    while (getAverage(A4) < .40 || getAverage(A4) > .30)
    {
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      left_motor.run(BACKWARD);
      right_motor.run(BACKWARD);
      delay(10000);
    }//end while at marker

  }//end case 3: two obstacles

}//close return to start
