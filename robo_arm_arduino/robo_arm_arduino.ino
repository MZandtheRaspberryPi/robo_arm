#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Robot Joint Motion Stuctures
struct armPos {
  int baseServoAngle;
  int midServoAngle ;
  int upperServoAngle ;
  int gripperServoAngle;
  int gripperRotationAngle;
  int desiredDelay ;
};

struct armPos armMotion;  // Joint Positions of arm


class RoboArm {
  // init to setup up the servos
  public: 
  // some of hte setup stuff like attaching pins can only be done within the arduino setup function i suspect
  // i need to init this as a global variable, and so doing this in a seperate function instead of in constructor
  void setup(int basePin, int midPin, int upperPin, int gripperPin, int gripperRotationPin,
             int joystickXPin, int joystickYPin, int joystickButtonPin) {
      this->joystickXPin = joystickXPin;
      this->joystickYPin = joystickYPin;
      this->joystickButtonPin = joystickButtonPin;

      pinMode(joystickYPin, INPUT);
      pinMode(joystickXPin, INPUT);
      pinMode(joystickButtonPin, INPUT_PULLUP); 
      
      // attaching servos to pins
      baseServo.attach(basePin);
      midServo.attach(midPin);
      upperServo.attach(upperPin);
      gripperServo.attach(gripperPin);
      gripperRotationServo.attach(gripperRotationPin);
      delay(500);
      // setting all servos to default position
      baseServo.write(90);
      midServo.write(90);
      upperServo.write(90);
      gripperServo.write(90);
      gripperRotationServo.write(90);
      delay(500);      
  }

  int servoParallelControl (int thePos, Servo theServo, int theSpeed ) {
      // this is a function to move a single servo by a small amount each time
      // and return whether we reach the final position
      // it is designed to be called many timeas to get to one position, by the function moveTo
      Serial.println("moving servo");

      int startPos = theServo.read();        //read the current pos
      int newPos = startPos;
      Serial.print(startPos);
      Serial.print(" ");
      Serial.println(newPos);
    
      //define where the pos is with respect to the command
      // if the current position is less that the actual move up
      if (startPos < (thePos - 5)) {
        newPos = newPos + 1;
        theServo.write(newPos);
        delay(theSpeed);
        return 0;
      }

      // else move the servo down
      else if (newPos > (thePos + 5)) {
        newPos = newPos - 1;
        theServo.write(newPos);
        delay(theSpeed);
        return 0;
      }

      // else we are at the desired position, or within 5 degrees
      else {
        return 1;
      }
   }

  void updateDisplay() {
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      return;
    }
    
    display.clearDisplay();
    

    int servoPos;
    servoPos = baseServo.read();
    display.setCursor(0, 0);     // Start at top-left corner
    display.print("Base: ");
    display.print(servoPos);

    servoPos = midServo.read();
    display.setCursor(0, 10);     // Start at top-left corner
    display.print("Mid: ");
    display.print(servoPos);

    servoPos = upperServo.read();
    display.setCursor(0, 20);     // Start at top-left corner
    display.print("Up: ");
    display.print(servoPos);

    servoPos = gripperServo.read();
    display.setCursor(65, 0);     // Start at top-left corner
    display.print("Grip: ");
    display.print(servoPos);

    servoPos = gripperRotationServo.read();
    display.setCursor(65, 10);
    display.print("Gr Rot: ");
    display.print(servoPos);

    display.setCursor(65, 20);
    display.print("Sel: ");
    display.print(selectedServo);
  
    display.display();
  }



  public: 
  void moveTo(struct armPos armMotion) {
  
    int status1 = 0;
    int status2 = 0;
    int status3 = 0;
    int status4 = 0;
    int status5 = 0;
    int done = 0 ;
  
    while ( done == 0) {
      //move all servos to the desired position
      //this loop will cycle through the servos sending each the desired position.
      //Each call will cause the servo to iterate about 1-5 degrees
      //the rapid cycle of the loop makes the servos appear to move simultaneously
      status1 = servoParallelControl(armMotion.baseServoAngle, baseServo, armMotion.desiredDelay);
      status2 = servoParallelControl(armMotion.midServoAngle, midServo, armMotion.desiredDelay);
      status3 = servoParallelControl(armMotion.upperServoAngle, upperServo, armMotion.desiredDelay);
      status4 = servoParallelControl(armMotion.gripperServoAngle, gripperServo, armMotion.desiredDelay);
      status5 = servoParallelControl(armMotion.gripperRotationAngle, gripperRotationServo, armMotion.desiredDelay);
  
      //continue until all have reached the desired position
      if (status1 == 1 & status2 == 1 & status3 == 1 & status4 == 1 & status5 == 1) {
        done = 1;
        updateDisplay();
      }
    }// end of while
  } //function end

  public:
  void listenJoystick(){
    int xValue;
    int yValue;
    int xMap;
    int yMap;
    int SW_state;

    SW_state = digitalRead(joystickButtonPin);
    xValue = analogRead(joystickXPin);
    yValue = analogRead(joystickYPin);
    if (xValue > 535 || xValue < 488) {
      xMap = map(xValue, 0, 1023, -1, 1);
    }
    else {
      xMap = 0;
    }

    if (yValue > 535 || yValue < 488) {
      yMap = map(yValue, 0, 1023, -1, 1);
    }
    else {
      yMap = 0;
    }

    // make sure button can only be pressed once ever 400 ms or so
    if (SW_state == 0 && millis() - lastButtonPress > 400) {
      // button press increments selected servo, resetting to 1 if we cycle through
      selectedServo += 1;
      if (selectedServo > 5) {
        selectedServo = 1;
      }
      lastButtonPress = millis();
      updateDisplay();
    }

    if (xMap == 0) {
      return;
    }
    // only using X values, as with my joytick too easy to unintentionally move both, so can only move 1 servo at a time with joystick
    int servoPos;
    int changeInPos;
    if (xMap > 0) {
      changeInPos = 6;
    }
    else {
      changeInPos = -6;
    }
    switch (selectedServo){
      case 1:
        servoPos = baseServo.read();
        servoPos += changeInPos;
        armMotion.baseServoAngle = servoPos;
        break;
      case 2:
        servoPos = midServo.read();
        servoPos += changeInPos;
        armMotion.midServoAngle = servoPos;
        break;
      case 3:
        servoPos = upperServo.read();
        servoPos += changeInPos;
        armMotion.upperServoAngle = servoPos;
        break;
      case 4:
        servoPos = gripperServo.read();
        servoPos += changeInPos;
        armMotion.gripperServoAngle = servoPos;
        break;
      case 5:
        servoPos = gripperRotationServo.read();
        servoPos += changeInPos;
        armMotion.gripperRotationAngle = servoPos;
        break;
    }
    Serial.println(servoPos);
     this->moveTo(armMotion);
  }

private:
  Servo baseServo;
  int baseServoDefaultPos = 90;
  int baseServoLowerLimit = 0;
  int baseServoUpperLimit = 180;
  Servo midServo;
  int midServoDefaultPos = 90;
  int midServoLowerLimit = 0;
  int midServoUpperLimit = 180;
  Servo upperServo;
  int upperServoDefaultPos = 90;
  int upperServoLowerLimit = 0;
  int upperServoUpperLimit = 180;
  Servo gripperServo;
  int gripperServoDefaultPos = 90;
  int gripperServoLowerLimit = 0;
  int gripperServoUpperLimit = 180;
  Servo gripperRotationServo;
  int gripperRotationServoDefaultPos = 90;
  int gripperRotationServoLowerLimit = 0;
  int gripperRotationServoUpperLimit = 180;

  int selectedServo = 1;
  int joystickXPin;
  int joystickYPin;
  int joystickButtonPin;

  unsigned long lastButtonPress;
};

RoboArm mz_robot;

void setup() {
  Serial.begin(57600);
  Serial.println("Startin setup");
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  mz_robot.setup(4, 5, 6, 7, 8, A0, A1, 2);
  armMotion.baseServoAngle = 90;
  armMotion.midServoAngle = 90;
  armMotion.upperServoAngle = 90;
  armMotion.gripperServoAngle = 130;
  armMotion.gripperRotationAngle = 90;
  armMotion.desiredDelay = 5;
  mz_robot.moveTo(armMotion);
  delay(2000);
  Serial.println("Done with setup");
}

void loop() {
  mz_robot.listenJoystick();
}
