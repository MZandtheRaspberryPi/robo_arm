#include <ros.h>
#include <robo_arm/arm_control.h>
#include "arm_movement.h"

ros::NodeHandle  nh;
char result[40]; // Buffer big enough for the below big log msg, generally

void arm_cb( const robo_arm::arm_control& arm_msg){
  xTarget = arm_msg.x;
  yTarget = arm_msg.y;
  zTarget = arm_msg.z;
  gripTarget = arm_msg.grip;
  duration = arm_msg.duration;
  nh.loginfo("Moving");

  while ((int)z != (int)zTarget || (int)x != (int)xTarget || (int)y != (int)yTarget || (int)grip != (int)gripTarget) {
    sprintf(result,"x: %d y: %d z: %d g: %d", (int)x, (int)y, (int)z, (int)grip);
    nh.loginfo(result);
    
    x = interpX.go(xTarget, duration);
    y = interpY.go(yTarget, duration);
    z = interpZ.go(zTarget, duration);
    grip = interpGrip.go(gripTarget, duration);

    moveArm(x, y, z, grip);
    
  }
  
}

ros::Subscriber<robo_arm::arm_control> sub("/robo_arm_teleop/arm_movement", arm_cb);

void readJoysticks(){
  nh.initNode();
  nh.subscribe(sub);
}

void setup() {
  setupObjs();
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {
    nh.spinOnce();
  }
}
