#include <Servo.h>

Servo myservo; 

void setup() {
  Serial.begin(115200);
  myservo.attach(5); 
}
void loop() {

  if(Serial.available()>0){
    char cmd = Serial.read();
    if(cmd=='a'){
      myservo.write(10);
      delay(100);
      
    }else if(cmd=='b'){
      myservo.write(150);
      delay(100);
    }
  }
  delay(500);
}
