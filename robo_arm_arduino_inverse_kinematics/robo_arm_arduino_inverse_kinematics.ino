#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ramp.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long currentMillis;
unsigned long previousMillis;

//*****************************
class Interpolation {  
public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;    

    int go(int input, int duration) {

      if (input != savedValue) {   // check for new data
          interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value  
    
      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
          myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
          interpolationFlag = 1;
      }
    
      int output = myRamp.update();               
      return output;
    }
};

Interpolation interpX;        // interpolation objects
Interpolation interpZ;

// current values
//int joystickLXPin = A0;
//int joystickLYPin = A1;
//int joystickLButtonPin = 2;
//
//int joystickRYPin = A3;
//int joystickRXPin = A2;
//int joystickRButtonPin = 3;

//pinMode(joystickLButtonPin, INPUT_PULLUP);
//pinMode(joystickRButtonPin, INPUT_PULLUP);

class RoboArm {

  void showArmLength(int x, int y, int z, int baseAngle, int elbowAngle, int shoulderAngle){

    if (this->lastBaseAngle == baseAngle && this->lastElbowAngle == elbowAngle && this->lastShoulderAngle == shoulderAngle) {
      return;
    }
    
    display.clearDisplay();
    
    display.setCursor(0, 0);     // Start at top-left corner
    display.print("X: ");
    display.print(x);
    display.print(" Y: ");
    display.print(y);
    display.print(" Z: ");
    display.print(z);

    display.setCursor(0, 8);     // Start at top-left corner
    display.print("elbow: ");
    display.print(elbowAngle);

    display.setCursor(0, 16);     // Start at top-left corner
    display.print("shoulder: ");
    display.print(shoulderAngle);

    display.setCursor(0, 24);     // Start at top-left corner
    display.print("base: ");
    display.print(baseAngle);
    
    display.display();
    lastShoulderAngle = shoulderAngle;
    lastElbowAngle = elbowAngle;
    lastBaseAngle = baseAngle;
  }

    // init to setup up the servos
  public: 
  // some of hte setup stuff like attaching pins can only be done within the arduino setup function i suspect
  // i need to init this as a global variable, and so doing this in a seperate function instead of in constructor
  void setup(int basePin, int midPin, int upperPin, int gripperPin, int gripperRotationPin,
             int joystickLXPin, int joystickLYPin, int joystickLButtonPin,
             int joystickRXPin, int joystickRYPin, int joystickRButtonPin) {

      if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        return;
      }
    
      display.clearDisplay();
      
      display.setCursor(0, 0);     // Start at top-left corner
      display.print("setting up arm");
      display.display();
      
      this->joystickLXPin = joystickLXPin;
      this->joystickLYPin = joystickLYPin;
      this->joystickLButtonPin = joystickLButtonPin;

      this->joystickRXPin = joystickRXPin;
      this->joystickRYPin = joystickRYPin;
      this->joystickRButtonPin = joystickRButtonPin;

      pinMode(joystickLYPin, INPUT);
      pinMode(joystickLXPin, INPUT);
      pinMode(joystickLButtonPin, INPUT_PULLUP);

      pinMode(joystickRXPin, INPUT);
      pinMode(joystickRYPin, INPUT);
      pinMode(joystickRButtonPin, INPUT_PULLUP);

      // attaching servos to pins
      baseServo.attach(basePin, baseServoLowerLimit, baseServoUpperLimit);
      shoulderServo.attach(midPin, shoulderLowerLimit, shoulderUpperLimit);
      elbowServo.attach(upperPin, elbowServoLowerLimit, elbowServoUpperLimit);
      gripperServo.attach(gripperPin, gripperServoLowerLimit, gripperServoUpperLimit);
      gripperRotationServo.attach(gripperRotationPin, gripperRotationServoLowerLimit, gripperRotationServoUpperLimit);
      delay(100);

      baseServo.writeMicroseconds(baseServoDefaultPos);
      shoulderServo.writeMicroseconds(shoulderDefaultPos);
      elbowServo.writeMicroseconds(elbowServoDefaultPos);
      gripperServo.writeMicroseconds(gripperServoClosedPos);
      gripperRotationServo.writeMicroseconds(gripperRotationServoDefaultPos);

  }

  float calculateElbowAngle(float upperLength, float lowerLength, float armLength){
      // C = arccos((a^2 + b^2 - c^2)/2ab)
      // elbowAngle = arccos((upperArm^2 + lowerArm^2 - armlength^2)/2*upperArm*lowerArm)
      // calculates in radians
      elbowAngleRadians = acos((sq(upperLength) + sq(lowerLength) - sq(armLength))/(2*upperLength*lowerLength));
      elbowAngleDegrees = elbowAngleRadians * (180/PI);
      return elbowAngleDegrees;
  }

  int elbowAngleToUs(float elbowAngleDegrees, int elbowOffsetUs){
      // firstly, get angles to us, then add in offset so that we have a us figure.
      // most servos have 11 ms per degree, so making into ms
      elbowAngleUs = (int) elbowAngleDegrees * 11;
      // we have an offset as 90 degrees, or 990us, isn't perpendicular to table in mine
//      elbowAngleUs += elbowOffsetUs;

      // as the degrees angle gets bigger, the microseconds need to decrease
      // because i've mounted my servo that way
      // to get to 110 degrees for example, the us has to decrease from what it was for 90.
      // so we find the change from default pos, and subtract it out from default pos
      // as calculated Us gets bigger, which will happen as angle gets bigger, we make final us smaller
//      elbowAngleUs = this->elbowServoDefaultPos - (elbowAngleUs - this->elbowServoDefaultPos);
      elbowAngleUs = this->elbowServoDefaultPos - (elbowAngleUs- 1000);
      return elbowAngleUs;
    
  }

  float calculateShoulderAngle(float upperLength, float lowerLength, float armLength){
      // B = arccos((a^2 + c^2 - b^2)/2*a*c)
      shoulderAngleRadians = acos((sq(upperLength) + sq(armLength) - sq(lowerLength))/(2*upperLength*armLength));
      shoulderAngleDegrees = shoulderAngleRadians * (180/PI);
      return shoulderAngleDegrees;
  }

  int shoulderAngleToUs(float shoulderAngleDegrees, int shoulderOffsetUs, float shoulderMs2){

      // most servos have 11 ms per degree, so making into ms
      shoulderAngleUs = (int) shoulderAngleDegrees * 11;
      // my servos aren't mounted so that the shoulder angle is 45degrees (in actuality 47 as my sides aren't equal of triangle
      // and it ain't isololes) when perpendicular to table, so adjust for that.
//      shoulderAngleUs += shoulderOffsetUs;

      // as the degrees angle gets bigger, the microseconds need to decrease
      // because i've mounted my servo that way
      // to get to 110 degrees for example, the us has to decrease from what it was for 90.
      // so we find the change from default pos, and subtract it out from default pos
      // as calculated Us gets bigger, which will happen as angle gets bigger, we make final us smaller
//      shoulderAngleUs = this->shoulderDefaultPos - (shoulderAngleUs - this->shoulderDefaultPos);
      shoulderAngleUs = this->shoulderDefaultPos - (shoulderAngleUs - 480) - shoulderMs2;
      return shoulderAngleUs;
  }

  int baseAngleToUs(float baseAngleDegrees){
    // map the degrees to the servo us of our base servo
    // when the base servo is like 700 us or so, it is pointing all the way to the right, 180 degrees
    // hence map bounds for us are flipped
    baseAngleUs = map(baseAngleDegrees, 0, 180, this->baseServoUpperLimit, this->baseServoLowerLimit);
    return baseAngleUs;
  }

  int readJoystickDirection(int pinToRead, int lowerMidCutoff, int upperMidCutoff, int lowerMap, int upperMap){
    xValue = analogRead(pinToRead);
    // often a bit of zone around middle joystick that you want to ignore to cut out noise
    if (xValue > upperMidCutoff || xValue < lowerMidCutoff) {
      xMap = map(xValue, 0, 1023, lowerMap, upperMap);
    }
    else {
      xMap = 0;
    }
    // our joysticks are placed on their side
    // so we get opposite values as we expect if we move joystick
    // moving x to the right should increase, moving y up should increase, so let's make it so
    xMap *= -1;
    return xMap;
  }

  bool readJoystickButton(int pinToRead, unsigned long lastButtonPress){
    SW_state = digitalRead(pinToRead);
    // make sure button can only be pressed once ever 400 ms or so
    if (SW_state == 0 && millis() - lastButtonPress > 400) {
      // button press increments selected servo, could resetting to 1 if we cycle through
      lastButtonPress = millis();
      return 1;
    }
    return 0;
  }

  void moveArm(float x, float y, float z, int gripperAngleUs){
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.print(y);
    Serial.print(" z: ");
    Serial.println(z);
    Serial.print("gripperAngleUs: ");
    Serial.println(gripperAngleUs);

    // solving for base servo angle first, given x and y
    float theta2 = atan2(y, x);
    float theta2Degrees = theta2 * (180/PI);
    float baseAngleDegrees = 180 - theta2Degrees;
    baseAngleUs = this->baseAngleToUs(baseAngleDegrees);
    Serial.print("baseAngleDegrees: ");
    Serial.print(baseAngleDegrees);
    Serial.print(" baseAngleUs: ");
    Serial.println(baseAngleUs);

    // distance to x,y,z point in just x and y space
    float f = sqrt(sq(x) + sq(y));
    Serial.print("f: ");
    Serial.println(f);

    // given f and z, we calculate theta 1 angle
    float theta1 = atan2(z, f);
    // given theta 1 and Z, we calculate the arm length
    // cos(theta1) = adj/hypotenuse
    this->armLength = (float)f / cos(theta1);
    
    float shoulderAngle2a = theta1 - 0.7853908;  // take away the default 45' offset (in radians)
    float shoulderAngle2aDegrees = shoulderAngle2a * (180/PI);    // degrees
    float shoulderMs2 = shoulderAngle2aDegrees * 11;

    elbowAngleDegrees = this->calculateElbowAngle(upperLength, lowerLength, armLength);
    Serial.print("elbowAngleDegrees: ");
    Serial.print(elbowAngleDegrees);
    elbowAngleUs = this->elbowAngleToUs(elbowAngleDegrees, elbowOffsetUs);
    Serial.print(" elbowAngleUs: ");
    Serial.println(elbowAngleUs);
    shoulderAngleDegrees = this->calculateShoulderAngle(upperLength, lowerLength, armLength);
    Serial.print("shoulderAngleDegrees: ");
    Serial.print(shoulderAngleDegrees);
    shoulderAngleUs = this->shoulderAngleToUs(shoulderAngleDegrees, shoulderOffsetUs, shoulderMs2);
    Serial.print(" shoulderAngleUs: ");
    Serial.println(shoulderAngleUs);
    this->showArmLength(x, y, z, baseAngleUs, elbowAngleUs, shoulderAngleUs);
    Serial.println();
    
    elbowServo.writeMicroseconds(elbowAngleUs);
    shoulderServo.writeMicroseconds(shoulderAngleUs);
    baseServo.writeMicroseconds(baseAngleUs);
    gripperServo.writeMicroseconds(gripperAngleUs);
    
  }

  public:
  void robotLoop(){
    
    changeXPos = readJoystickDirection(this->joystickLXPin,  425, 600, -1, 1);
    changeYPos = readJoystickDirection(this->joystickLYPin,  425, 600, -1, 1);
    if (readJoystickButton(this->joystickLButtonPin, this->lastLButtonPress)){
      Serial.println("L Button Pressed");
    }

    changeGripperPos = readJoystickDirection(this->joystickRXPin,  425, 600, -5, 5);
    changeZPos = readJoystickDirection(this->joystickRYPin,  425, 600, -1, 1);
    if (readJoystickButton(this->joystickRButtonPin, this->lastRButtonPress)){
      Serial.println("R Button Pressed");
    }

    x += changeXPos;
    y += changeYPos;
    z += changeZPos;

    if (gripperAngleUs + changeGripperPos > gripperServoLowerLimit && gripperAngleUs + changeGripperPos < gripperServoUpperLimit) {
      gripperAngleUs += changeGripperPos;
    }

    this->moveArm(x, y, z, gripperAngleUs);
  }

private:
  // defaults in microseconds
  Servo baseServo;
  // default is 90 degrees
  // 1500 microseconds
  int baseServoDefaultPos = 1530;
  // this is gripper pointing to right of base
  int baseServoLowerLimit = 690;
  // this is gripper pointing to left of base
  int baseServoUpperLimit = 2300;
  Servo shoulderServo;
  // 780 is about straight up and down, ie, 90 degrees with table
  // as Ms gets smaller, arm moves towards joysticks
  int shoulderDefaultPos = 800;
  int shoulderLowerLimit = 650;
  // 100 degrees
  int shoulderUpperLimit = 1500;
  // 45 degrees at 11us per degree is 495us
  // my shoulder is at 45 degree position when us is 780
  // therefore, add an offset of 800 - 495 to the calculated shoulderUs
  int shoulderOffsetUs = shoulderDefaultPos - 495;
  Servo elbowServo;
  // 1730 is about parallel to the table
  // elbow servo mounted so that as microseconds decrease, it rotates away from robo arm base (ie, upward)
  int elbowServoDefaultPos = 1730;
  int elbowServoLowerLimit = 790;
  int elbowServoUpperLimit = 2210;
  // 90 degrees at 11us per degree is 990us
  // my elbow is at 90 degree position when us is 1730
  // therefor, add an offset of 1730-990 to the calculated eblowUs
  int elbowOffsetUs = 1730 - 990;
  Servo gripperServo;
  // 1640 is open
  // 2080 is closed about
  int gripperServoOpenPos = 1640;
  int gripperServoClosedPos = 2080;
  int gripperServoDefaultPos = gripperServoOpenPos;
  int gripperServoLowerLimit = 1490;
  int gripperServoUpperLimit = 2200;
  Servo gripperRotationServo;
  // 
  int gripperRotationServoDefaultPos = 1220;
  int gripperRotationServoLowerLimit = 700;
  int gripperRotationServoUpperLimit = 1890;

  int selectedServo = 1;
  int joystickLXPin;
  int joystickLYPin;
  int joystickLButtonPin;

  int joystickRXPin;
  int joystickRYPin;
  int joystickRButtonPin;
  
  unsigned long lastLButtonPress;
  unsigned long lastRButtonPress;

  int xValue;
  int yValue;
  int xMap;
  int yMap;
  int SW_state;

  int changeXPos;
  int changeYPos;
  int changeZPos;
  int changeGripperPos;

  // c^2 = a^2 + b^2 -2abCos(C)
  // lower is the arm between gripper and upper mid servo
  float lowerLength = 170;
  // upper is the arm between mid servo and upper mid servo
  float upperLength = 158;
  
  float elbowAngleDegrees;
  float elbowAngleRadians;
  float shoulderAngleDegrees;
  float shoulderAngleRadians;

  int elbowAngleUs;
  int shoulderAngleUs;
  int baseAngleUs;
  int gripperAngleUs = gripperServoDefaultPos;
  
  // armLength is from mid servo to tip of gripper servo
  // we'll set this at the start and use it to solve for the angles in the upper arm
  // 233 is about the hypotenuse that sets shoulder to 90 degrees and elbow to 45 degrees, ie upper arm perpendicular to table
  float armLength = 233;

  // elbow offset is because I didn't mount the servo so that when I commanded it to 90 it would be
  // perpendicular to the table. I mounted it so that when it is at 122 it is perpendicular to the table.
  int elbowOffset = 122-90;
  // similar with shoulder offset
  int shoulderOffset = 18-47;


  // this is for display, so we don't have to update each loop if hasn't changed:
  int lastBaseAngle;
  int lastElbowAngle;
  int lastShoulderAngle;


  // setting x, y, z coords in mm
  float x = 0;
  float y = 170;
  float z = 158;
};

RoboArm mz_robot;

void setup() {
  Serial.begin(57600);
  Serial.println("Starting setup");
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  // basePin, int midPin, int upperPin, int gripperPin, int gripperRotationPin
  mz_robot.setup(4, 5, 7, 8, 6, A0, A1, 2, A2, A3, 3);
  delay(2000);
  Serial.println("Done with setup");
  display.clearDisplay();
}

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= 10) {  // start timed loop
      previousMillis = currentMillis;
      mz_robot.robotLoop(); 
  }
  
}
