void scannerSetup(){
  while(RSens == 1 || BSens == 1 || DSens == 1 ||LSens == 1 || LeftEncoderCount < 20 || RightEncoderCount  < 20){
      if (RightEncoderCount > 20){
        Serial.println("Right Encoder Working");
      }
      if (LeftEncoderCount > 20){
        Serial.println("Left Encoder Working");
      }
      if (getAverage(LSensor) < 2){
        Serial.println("Left Sensor Working");
        LSens=0;
      }
      if (getAverage(RSensor) < 2){
        Serial.println("Right Sensor Working");
        RSens=0;
      }
      if (getAverage(sensor) > 0.23 && getAverage(sensor) < 0.29){
        Serial.println("Bottom Sensor Working");
        BSens=0;
      }
      if (getAverageDistance() < 50){
        Serial.println("Distance Sensor Working");
        DSens=0;
      }
  }
  Serial.println("Sensor Test done");
  RightEncoderCount = 0;
  LeftEncoderCount = 0;
  delay(2000);
}
