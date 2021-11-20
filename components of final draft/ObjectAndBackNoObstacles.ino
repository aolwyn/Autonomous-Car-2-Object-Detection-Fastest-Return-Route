//Adam Bayley 20176309
//week 4 activity 1 individual task submission file


#include <AFMotor.h>
#include <PID_v2.h> 


#define LEncoderPin 3  //left encoder
#define REncoderPin 2  //right encoder
#define LSensor A0  //left front sensor
#define RSensor A5  //right front sensor
#define trig A1  //ultrasonic sensor trigger pin
#define echo A2  //ultrasonic sensor echo pin

#define sensor A4 //botton sensor

AF_DCMotor left_motor(1, MOTOR34_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR12_1KHZ); // right motor to M3 on motor control board


int lastTimeL = 0;
int lastTimeR = 0;
int LeftEncoderCount = 0;
int RightEncoderCount = 0;
int DelayTime = 1000;
int PrevRightEncoderCount = 0; 
int PrevLeftEncoderCount = 0; 

//for PIV control:
double EncDelta=0;
double OutDelta=-20.0;
double EncDeltaSet=0;
double Kp = 0.5, Ki = 0.5, Kd = 0.1; // starting PID controller settings
int LSens = 1, RSens = 1, BSens = 1, DSens = 1, ABC = 1, XYZ = 1;

int RPWM = 0;
int LPWM = 0;
PID DiffSpeed(&EncDelta, &OutDelta, &EncDeltaSet,Kp,Ki,Kd, DIRECT);

/*PSUEDOCODE OF FUNCTION AND ASSUMPTIONS:
 * NOTE: I HAVE 2 MARKERS. 1 FOR START, 1 FOR END.
 * NOTE 2: my motors are reversed. FORWARD = BACKWARD.
 * CODE:
 * while flag != 1
 * move forward
 * if ultasonic sensor senses something, slow down, break, and reactObstacles
 * break out, 
 * do the 180, 
 * incremement flag, 
 * go back to start
 * 
 * alternative solution: use encoders to count around obstacles to then go back on track.
 * no time to do the better solution so this one will work too (i tested)
 */

void setup() {

  attachInterrupt(1, countLEncoder, RISING); //calls on any CHANGE on pin 3 (interrupt #1, soldered to Pin3) 
  attachInterrupt(0, countREncoder, RISING); //calls on any CHANGE on pin 2 (interrupt #0, connected to header on Pin2) 
  interrupts();
  
  Serial.begin (115200);    // set up Serial library at 115200 bps

 //setup input and output pins
  pinMode(LSensor, INPUT);
  pinMode(RSensor, INPUT);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(sensor, INPUT);


  //PID setting speeds and maximum outputs
  DiffSpeed.SetMode(AUTOMATIC);
  DiffSpeed.SetOutputLimits(-150,150);
  
  // Set motor direction
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

  //scannerSetup();
}



void loop() 
{
  int loopFlag = 1;
 
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  while(loopFlag < 3)
  {
 if(loopFlag == 1)
 {
  Serial.println("First Round, on way to 2m marker");
 }
 else if(loopFlag == 2)
 {
  Serial.println("On Route to Starting Position.");
 }

   adjustSpeed(170);
   int getDistanceVal = getDistance();
if(getDistanceVal < 50)
{
while(getDistanceVal<50)
{
  getDistanceVal = getDistance();
  adjustSpeed(130);
}//close while

reactObstacles();
}//end if getdstanceval <50
reactObstacles();
 if (getVoltage(sensor) > 0.40 && getVoltage(sensor) < 0.48){
TurnAround(); 
  Serial.println("At 2m marker, turning around");
loopFlag++;

 }//end if at marker
}//end while loop flag < 3
}//end void loop





void countLEncoder(){ // interrupt function for left encoder
 if (micros() - lastTimeL > 100){
 LeftEncoderCount++;
 lastTimeL = micros();
 }//end if
}//end countLEncoder 

void countREncoder(){ // interrupt function for right encoder
 if (micros() - lastTimeR > 100){
 RightEncoderCount++;
 lastTimeR = micros();
 }//end if
}//end countREncoder


//function for getting distance based on ultrasonic sensor
float getDistance(){
  digitalWrite(trig, LOW);
  delayMicroseconds(10);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  float duration = pulseIn(echo,HIGH,50000000);
  float velocity = 0.0343;
  float distance = duration*velocity/(2);
  return distance;
}//end getDistance

//function for getting and adjusting speed
void adjustSpeed(int i){
  
  EncDelta = (double)(1.0*LeftEncoderCount - 1.0*RightEncoderCount);
  DiffSpeed.Compute(); 
  RPWM = (int)round(i-OutDelta);
  LPWM = (int)round(i+OutDelta);
  
  left_motor.setSpeed(LPWM);
  right_motor.setSpeed(RPWM); 
}//end adjustSpeed


//function for getting voltage based on calculation given
float getVoltage(int pin) {
  float voltage = 5.0 * analogRead(pin) / 1024;
  return voltage;
}//end getVoltage

//function for checking motor capabilities (if they work,) before use; utilizes flags to see if it works
//and then encoder differences for the motors themselves
void scannerSetup(){
  while(RSens == 1 || BSens == 1 || DSens == 1 ||LSens == 1 || LeftEncoderCount < 20 || RightEncoderCount <20){
      if (getVoltage(LSensor) < 2){
        Serial.println("Left IR Sensor Working");
        LSens=0;
      }
      Serial.println(getVoltage(RSensor));
      if (getVoltage(RSensor) < 2){
        Serial.println("Right IR Sensor Working");
        RSens=0;
      }
      Serial.println(getVoltage(sensor));
      if (getVoltage(sensor) > 0.40 && getVoltage(sensor) < 0.48){
        Serial.println("Bottom Sensor Working");
        BSens=0;
      }
      Serial.println(getDistance());
      if (getDistance() < 50){
        Serial.println("Distance Sensor Working");
        DSens=0;
      }
      if( RightEncoderCount > 20){
         Serial.println("Left Encoder Sensor Working");
      }
      if( LeftEncoderCount > 20){
         Serial.println("Right Encoder Sensor Working");
      }
  }
  EncoderReset();
  delay(2000);
} //end ScannerSetup()

void EncoderReset()
{
    LeftEncoderCount = 0;
  RightEncoderCount = 0;
}//end EncoderReset

void Backup()
{
   //Backup
    left_motor.run(FORWARD);
    right_motor.run(FORWARD);
    for(int i = 0; i < 10; i++){
    adjustSpeed(150);
    delay(100);
}//end for
} //end backup

void LeftTurn()
{
    //turn left
    left_motor.run(BACKWARD);
    right_motor.run(BACKWARD);    
    right_motor.setSpeed(200);
    left_motor.setSpeed(0);
    delay(600);
    right_motor.setSpeed(0);
}//end LeftTurn



void RightTurn()
{
    //turn right
    left_motor.run(BACKWARD);
    right_motor.run(BACKWARD);    
    right_motor.setSpeed(0);
    left_motor.setSpeed(200);
    delay(600);
    left_motor.setSpeed(0);
}//end RightTurn



void TurnAround()
{
    //180 turn
    left_motor.run(BACKWARD);
    right_motor.run(BACKWARD);    
    right_motor.setSpeed(0);
    left_motor.setSpeed(200);
    delay(1000);
    left_motor.setSpeed(0);
}//end TurnAround



void reactObstacles()
{
  //if both sense something
  if(getVoltage(LSensor) < 2 && getVoltage(RSensor)<2)
  {
   Backup();
   TurnAround();
  }
  
  //if left senses something
  else if (getVoltage(LSensor) < 2)
  {
   Backup();
   RightTurn();
  }
  //if right senses something
    else if (getVoltage(RSensor) < 2)
    {
     Backup();
     LeftTurn();
    }
EncoderReset();
  delay(500);
} //end reactObstacles()
