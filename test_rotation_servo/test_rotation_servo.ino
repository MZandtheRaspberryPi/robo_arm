#include<Servo.h>

Servo testServo;
int servoAngle = 40;
    
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  testServo.attach(5);
  delay(1000);
  testServo.write(40);
  delay(1000);

}

void loop() {
    int xValue;
    int xMap;

    servoAngle = testServo.read();
    xValue = analogRead(A0);

    if (xValue > 600 || xValue < 430) {
      xMap = map(xValue, 0, 1023, -1, 1);
    }
    else {
      xMap = 0;
      return;
    }

    // only using X values, as with my joytick too easy to unintentionally move both, so can only move 1 servo at a time with joystick
    int changeInPos;
    if (xMap > 0) {
      changeInPos = 1;
    }
    else {
      changeInPos = -1;
    }

    servoAngle += changeInPos;
    Serial.println(servoAngle);
    // trying to command servo dangerously close to its limits
    if (servoAngle > 170 || servoAngle < 10) {
      servoAngle -= changeInPos;
      return;
    }
    testServo.write(servoAngle);
    delay(150);

}
