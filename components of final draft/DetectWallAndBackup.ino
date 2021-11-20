//code moves forward until it detects an obstacle. backs up and pivots accordingly. 
#include <AFMotor.h>
#define delayTime 1000    

AF_DCMotor left_motor(1, MOTOR12_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR34_1KHZ); // right motor to M3 on motor control board

void setup() {

  pinMode(A0, INPUT); //left 
  pinMode(A5, INPUT); //right

  Serial.begin (115200);    // set up Serial library at 115200 bps

  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

}

void loop() {
  // put your main code here, to run repeatedly:

 Serial.println("Forward");
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);

  reactObstacles();

}//end void loop()


void reactObstacles(){

   if (digitalRead(A0) == LOW && digitalRead(A5) == HIGH)
   {
    Backup();
    pivot(1);
  }
  else if (digitalRead(A5) == LOW && digitalRead(A0) == HIGH)
  {
    Backup();
    pivot(2);
  }
  else if (digitalRead(A5) == LOW && digitalRead(A0) == LOW)
  {
    Backup();
    pivot(3);
  }
  else
{
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);
  left_motor.setSpeed(200);
  right_motor.setSpeed(200);
  }
  
} //end void reactObstacles();

void backup() {
  Serial.println("Backward");
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(100);
  right_motor.setSpeed(100);
  delay(1000); 

}//end void backup();


  
//left = 1, right = 2, left and right = 3
void pivot(int Direction){
  if(Direction == 1){
    left_motor.run(BACKWARD);
    right_motor.run(FORWARD);
    left_motor.setSpeed(200);
    right_motor.setSpeed(200); 
    delay(400);
      
  }else if(Direction == 2){
    left_motor.run(FORWARD);
    right_motor.run(BACKWARD);   
    left_motor.setSpeed(200);
    right_motor.setSpeed(200);
    delay(400); 
      
  }else if(Direction == 3){
    left_motor.run(FORWARD);
    right_motor.run(BACKWARD);
    left_motor.setSpeed(200);
    right_motor.setSpeed(200); 
    delay(400);
       
  }
}//end pivot()
