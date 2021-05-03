#include <Servo.h>
#include <Ramp.h>

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
Interpolation interpY;
Interpolation interpGrip;

int basePin = 4;
int midPin = 5;
int upperPin = 7;
int gripperPin = 8;
int gripperRotationPin = 6;
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
int shoulderUpperLimit = 1500;
Servo elbowServo;
// 1730 is about parallel to the table
// elbow servo mounted so that as microseconds decrease, it rotates away from robo arm base (ie, upward)
int elbowServoDefaultPos = 1730;
int elbowServoLowerLimit = 790;
int elbowServoUpperLimit = 2210;
Servo gripperServo;
// 1640 is open
// 2080 is closed about
int gripperServoOpenPos = 1640;
int gripperServoClosedPos = 2080;
int gripperServoDefaultPos = gripperServoOpenPos;
int gripperServoLowerLimit = 1490;
int gripperServoUpperLimit = 2200;
Servo gripperRotationServo;
// this one isn't used in kinematics
int gripperRotationServoDefaultPos = 1220;
int gripperRotationServoLowerLimit = 700;
int gripperRotationServoUpperLimit = 1890;

// c^2 = a^2 + b^2 -2abCos(C)
// lower is the arm between gripper and upper mid servo
float lowerLength = 170;
// upper is the arm between mid servo and upper mid servo
float upperLength = 158;
// armLength is from mid servo to tip of gripper servo
// we'll set this at the start and use it to solve for the angles in the upper arm
// 233 is about the hypotenuse that sets shoulder to 90 degrees and elbow to 45 degrees, ie upper arm perpendicular to table
float armLength = 233;


float elbowAngleDegrees;
float elbowAngleRadians;
float shoulderAngleDegrees;
float shoulderAngleRadians;

int elbowAngleUs;
int shoulderAngleUs;
int baseAngleUs;

// this is for display, so we don't have to update each loop if hasn't changed:
int lastBaseAngle;
int lastElbowAngle;
int lastShoulderAngle;


// setting x, y, z coords in mm to start with. This is a default pos that won't break anything.
float x = 0;
float y = 170;
float z = 158;
float grip = gripperServoDefaultPos;

float xTarget;
float yTarget;
float zTarget;
float gripTarget;

int duration;

unsigned long currentMillis;
unsigned long previousMillis;

void setupObjs(){
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

int elbowAngleToUs(float elbowAngleDegrees){
      // firstly, get angles to us, then add in offset so that we have a us figure.
      // most servos have 11 ms per degree, so making into ms
      elbowAngleUs = (int) elbowAngleDegrees * 11;
      elbowAngleUs = elbowServoDefaultPos - (elbowAngleUs- 1000);
      return elbowAngleUs;
}


float calculateShoulderAngle(float upperLength, float lowerLength, float armLength){
      // B = arccos((a^2 + c^2 - b^2)/2*a*c)
      shoulderAngleRadians = acos((sq(upperLength) + sq(armLength) - sq(lowerLength))/(2*upperLength*armLength));
      shoulderAngleDegrees = shoulderAngleRadians * (180/PI);
      return shoulderAngleDegrees;
  }

int shoulderAngleToUs(float shoulderAngleDegrees, float shoulderMs2){
    // most servos have 11 ms per degree, so making into ms
    shoulderAngleUs = (int) shoulderAngleDegrees * 11;
    shoulderAngleUs = shoulderDefaultPos - (shoulderAngleUs - 480) - shoulderMs2;
    return shoulderAngleUs;
}

int baseAngleToUs(float baseAngleDegrees){
  // map the degrees to the servo us of our base servo
  // when the base servo is like 700 us or so, it is pointing all the way to the right, 180 degrees
  // hence map bounds for us are flipped
  baseAngleUs = map(baseAngleDegrees, 0, 180, baseServoUpperLimit, baseServoLowerLimit);
  return baseAngleUs;
}


void moveArm(float x, float y, float z, float gripperUs){

    // solving for base servo angle first, given x and y
    float theta2 = atan2(y, x);
    float theta2Degrees = theta2 * (180/PI);
    float baseAngleDegrees = 180 - theta2Degrees;
    baseAngleUs = baseAngleToUs(baseAngleDegrees);

    // distance to x,y,z point in just x and y space
    float f = sqrt(sq(x) + sq(y));

    // given f and z, we calculate theta 1 angle
    float theta1 = atan2(z, f);
    // given theta 1 and Z, we calculate the arm length
    // cos(theta1) = adj/hypotenuse
    armLength = (float)f / cos(theta1);
    
    float shoulderAngle2a = theta1 - 0.7853908;  // take away the default 45' offset (in radians)
    float shoulderAngle2aDegrees = shoulderAngle2a * (180/PI);    // degrees
    float shoulderMs2 = shoulderAngle2aDegrees * 11;

    elbowAngleDegrees = calculateElbowAngle(upperLength, lowerLength, armLength);
    elbowAngleUs = elbowAngleToUs(elbowAngleDegrees);
    shoulderAngleDegrees = calculateShoulderAngle(upperLength, lowerLength, armLength);
    shoulderAngleUs = shoulderAngleToUs(shoulderAngleDegrees, shoulderMs2);
    
    elbowServo.writeMicroseconds(elbowAngleUs);
    shoulderServo.writeMicroseconds(shoulderAngleUs);
    baseServo.writeMicroseconds(baseAngleUs);
    gripperServo.writeMicroseconds((int)gripperUs);
    
  }
