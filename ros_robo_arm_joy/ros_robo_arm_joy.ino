#include <ros.h>
#include <robo_arm/joysticks.h>
#include "robo_joysticks.h"

ros::NodeHandle  nh;

robo_arm::joysticks joy_msg;
ros::Publisher joy_pub("/robo_arm_teleop/joysticks", &joy_msg);

void readJoysticks(){
    lxValue = readJoystickDirection(joystickLXPin,  425, 600, -1, 1);
    lyValue = readJoystickDirection(joystickLYPin,  425, 600, -1, 1);
    lbuttonPressed = readJoystickButton(joystickLButtonPin, lastLButtonPress);
    joy_msg.left_x = lxValue;
    joy_msg.left_y = lyValue;
    joy_msg.left_button = lbuttonPressed;
    
    rxValue = readJoystickDirection(joystickRXPin,  425, 600, -1, 1);
    ryValue = readJoystickDirection(joystickRYPin,  425, 600, -1, 1);
    rbuttonPressed = readJoystickButton(joystickRButtonPin, lastRButtonPress);
    joy_msg.right_x = rxValue;
    joy_msg.right_y = ryValue;
    joy_msg.right_button = rbuttonPressed;
}

void setup() {
  pinMode(joystickLYPin, INPUT);
  pinMode(joystickLXPin, INPUT);
  pinMode(joystickLButtonPin, INPUT_PULLUP);

  pinMode(joystickRXPin, INPUT);
  pinMode(joystickRYPin, INPUT);
  pinMode(joystickRButtonPin, INPUT_PULLUP);

      
  nh.initNode();
  nh.advertise(joy_pub);

}

void loop() {
  readJoysticks();
  joy_pub.publish( &joy_msg );
  nh.spinOnce();
  delay(1);


}
