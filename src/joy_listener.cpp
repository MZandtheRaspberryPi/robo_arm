#include "ros/ros.h"
#include "robo_arm/joysticks.h"
#include<geometry_msgs/Twist.h>
#include<control_msgs/GripperCommand.h>


class Listener 
{
  public:
    Listener(ros::Publisher& vel_pub, ros::Publisher& grip_pub){
      this->vel_pub = vel_pub;
      this->grip_pub = grip_pub;
    }

    ros::Publisher vel_pub;
    ros::Publisher grip_pub;
    void joystickCallback(const robo_arm::joysticks::ConstPtr& msg);
};


void Listener::joystickCallback(const robo_arm::joysticks::ConstPtr& msg)
{
  if (msg->left_x != 0 || msg->left_y != 0 || msg->right_x != 0) {
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = msg->left_x;
  vel_msg.linear.y = msg->left_y;
  vel_msg.linear.z = msg->right_x;
  this->vel_pub.publish(vel_msg);
  }

  if (msg->right_y != 0) {
  control_msgs::GripperCommand grip_msg;
  grip_msg.position = msg->right_y;
  this->grip_pub.publish(grip_msg);
  }

  ros::spinOnce();

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "joy_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Publisher grip_pub = n.advertise<control_msgs::GripperCommand>("/grip_cmd", 1000);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  Listener listener = Listener(vel_pub, grip_pub);
  ros::Subscriber sub = n.subscribe("robo_arm_teleop/joysticks", 10, &Listener::joystickCallback, &listener);

  ros::Rate loop_rate(10);
  /*
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
