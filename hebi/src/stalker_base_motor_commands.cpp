#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Twist.h>
#include <urdf/model.h>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

#include <sstream>
#include <string>
#include <vector>
#include <math.h>

//const int MAX_RAW_VELOCITY = 50;
const int MAX_RAW_VELOCITY = 5;

/**
 * A short function to get name and family from a name split with a "|".
 */
bool split(const std::string &orig, std::string &name, std::string &family)
{
  std::stringstream ss(orig);
  if (!std::getline(ss, name, '|'))
    return false;
  std::getline(ss, family);
  return true;
}

// Global 'group' pointer so we can access this in a callback...ugly, but until
// we class-ify the node this will work.
hebi::Group* group_g = NULL;

/**
 * Does kinematics to send omnidrive translation commands to the wheels
 */
// int sendWheelTranslationCommands(double velocity, double angle)
// {
//   std::cout << "Velocity:\n\t" << velocity << " Angle:" << angle << "\n";

//   // Verify angle is from 0 - 2PI radians.
//   if ((angle > (2*M_PI)) || (angle < 0)) {
//     ROS_INFO("Invalid Angle!\n");
//     return(-1);
//   }

//   // Sanity check velocity.
//   if (velocity > MAX_RAW_VELOCITY) {
//     ROS_INFO("Velocity above max range\n");
//     velocity = MAX_RAW_VELOCITY;
//   }

//   // Sanity check velocity.
//   if (velocity < 0) {
//     ROS_INFO("Velocity out of range (0)\n");
//     velocity = 0;
//   }

//   // velocity = [-1,1]
//   double TWIST_RATE = 0.01; // [-1,1]
//   double wheel_front_right, wheel_rear_left, wheel_rear_right;

//   if ((velocity == 0) || (velocity == -0)){
//     wheel_front_right = 0;
//     wheel_rear_left =  0;
//     wheel_rear_right = 0;
//   } else {
//     wheel_front_right = -1 * velocity * cos(angle + (M_PI/4)) - TWIST_RATE;
//     wheel_rear_left =  velocity * cos(angle + (M_PI/4)) + TWIST_RATE;
//     wheel_rear_right = -1 * velocity * sin(angle + (M_PI/4)) - TWIST_RATE;
//   }


//   hebi::GroupCommand cmd(3);
//   cmd[0].actuatorCommand().setVelocity(wheel_rear_right);
//   cmd[1].actuatorCommand().setVelocity(wheel_front_right);
//   cmd[2].actuatorCommand().setVelocity(wheel_rear_left);
//   group_g->sendCommand(cmd);
//   std::cout << "Wheel commands (translation):\n\t" << wheel_front_right <<
//             "\n\t" << wheel_rear_left << "\t" << wheel_rear_right << ".\n";

//   return(1);
// }

/**
 * Does traditional skid steer commands for the wheels
 */
int sendWheelSkidCommands(double velocityLeft, double velocityRight)
{
  std::cout << "Skid Command Velocity  -  Left:" << velocityLeft << "\t VelocityRight:" << velocityRight << std::endl;

  // Sanity check velocity.
  if (fabs(velocityLeft) > MAX_RAW_VELOCITY) {
    ROS_INFO("Velocity (skid-left) out of range\n");
    return(-1);
  }
  // Sanity check velocity.
  if (fabs(velocityRight) > MAX_RAW_VELOCITY) {
    ROS_INFO("Velocity (skid-right) out of range\n");
    return(-1);
  }

  double wheel_front_left, wheel_front_right, wheel_rear_left, wheel_rear_right;
  if (((velocityLeft == 0) || (velocityLeft == -0)) && ((velocityRight == 0) || (velocityRight == -0))) {
    wheel_front_left = 0;
    wheel_front_right = 0;
    wheel_rear_left =  0;
    wheel_rear_right = 0;
  } else {
    wheel_front_left = -velocityLeft;
    wheel_front_right = 1 * velocityRight;
    wheel_rear_left =  -velocityLeft;
    wheel_rear_right = 1 * velocityRight;
  }

  hebi::GroupCommand cmd(4);
  cmd[0].actuatorCommand().setVelocity(wheel_front_left);
  cmd[1].actuatorCommand().setVelocity(wheel_rear_left);
  cmd[2].actuatorCommand().setVelocity(wheel_front_right);
  cmd[3].actuatorCommand().setVelocity(wheel_rear_right);
  group_g->sendCommand(cmd);
  std::cout << "Wheel commands (skid):\n\t" << wheel_rear_right << "\t" << wheel_rear_left << std::endl;

  return(1);
}

/**
 * Callback for joystick commands
 */

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
double velocityR, velocityL;

  // Analog inputs
  const int LEFT_UP = 1;
  const int RIGHT_UP = 4;

  // Discrete inputs
  const int TOP_RIGHT_BUTTON = 5;
  const int TOP_LEFT_BUTTON = 4;
  const int A_BUTTON = 0;

  const int DEADBAND = 0.05;

  // High speed, requires button to be depressed to send commands
  if (msg->buttons[TOP_LEFT_BUTTON] > 0) {
    velocityL = msg->axes[LEFT_UP] * MAX_RAW_VELOCITY;
    if (abs(msg->axes[LEFT_UP]) < DEADBAND)  {
      velocityL = 0;
      }
    velocityR = msg->axes[RIGHT_UP] * MAX_RAW_VELOCITY;
    if (abs(msg->axes[RIGHT_UP]) < DEADBAND)  {
      velocityR = 0;
      }
  // Low speed, requires top right button to be depressed to send commands
  } else if (msg->buttons[TOP_RIGHT_BUTTON] > 0) {
    velocityL = msg->axes[LEFT_UP] * MAX_RAW_VELOCITY * 0.25;
    if (abs(msg->axes[LEFT_UP]) < DEADBAND)  {
      velocityL = 0;
      }
    velocityR = msg->axes[RIGHT_UP] * MAX_RAW_VELOCITY * 0.25;
    if (abs(msg->axes[RIGHT_UP]) < DEADBAND)  {
      velocityR = 0;
      }
  // Driving config where holding A and pushing the left joystick will allow straight movement only.
  } else if (msg->buttons[A_BUTTON] > 0) {
    velocityL = msg->axes[LEFT_UP] * MAX_RAW_VELOCITY;
    velocityR = msg->axes[LEFT_UP] * MAX_RAW_VELOCITY;
    if (abs(msg->axes[LEFT_UP]) < DEADBAND) {
      velocityL = 0;
      velocityR = 0;
      }  
  } else {
    velocityR = 0;
    velocityL = 0;
  }

 // if (msg->axes[DPAD_UP] > 0){  //drive forwards
    // std::cout << "forward mode\n";
    // velocity = MAX_RAW_VELOCITY/3.0;
 //  	sendWheelSkidCommands(velocity, velocity);
 //   } else if (msg->axes[DPAD_UP] < 0){  //drive reverse
    // std::cout << "reverse mode\n";
    // velocity = MAX_RAW_VELOCITY/3.0;
 //  	sendWheelSkidCommands(-1*velocity, -1*velocity);
 //   } else if (msg->axes[DPAD_SIDE] > 0){  //drive left pointturn
    // std::cout << "left mode\n";
    // velocity = MAX_RAW_VELOCITY/3.0;
 //  	sendWheelSkidCommands(-1*velocity, velocity);
 //   } else if (msg->axes[DPAD_SIDE] < 0){  //drive right pointturn
    // std::cout << "right  mode\n";
    // velocity = MAX_RAW_VELOCITY/3.0;
 //  	sendWheelSkidCommands(velocity, -1*velocity);
 //   } else { // Stop motion
    // std::cout << "stop mode\n";
 //  	velocity = 0;
 //  	sendWheelSkidCommands(velocity, velocity);
  sendWheelSkidCommands(velocityL, velocityR);
}

/*
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  double velocity, angle;

  const int LEFT_JOY_SIDE = 0;
  const int LEFT_JOY_UP = 1;
  const int RIGHT_JOY_SIDE = 3;
  const int RIGHT_JOY_UP = 4;
  const int DPAD_SIDE = 6;
  const int DPAD_UP = 7;

  const int DEADBAND = 0.8;

  if (msg->axes[LEFT_JOY_UP] > DEADBAND){  //Crazy mecanum wheel motions
    std::cout << "Mecanum mode\n";
    velocity = fabs(msg->axes[LEFT_JOY_UP]) * MAX_RAW_VELOCITY;
    if ((fabs(msg->axes[RIGHT_JOY_UP]) > DEADBAND) || (fabs(msg->axes[RIGHT_JOY_SIDE]) > DEADBAND)) {
        angle = atan2(-1*msg->axes[RIGHT_JOY_SIDE],msg->axes[RIGHT_JOY_UP]);
        if (angle < 0) angle+=2*M_PI;
    } else {
        angle = 0;
    }
    sendWheelTranslationCommands(velocity, angle);
   } else if (msg->axes[DPAD_UP] > 0){  //drive forwards
    std::cout << "forward mode\n";
    velocity = MAX_RAW_VELOCITY/3.0;
    sendWheelSkidCommands(velocity, velocity);
   } else if (msg->axes[DPAD_UP] < 0){  //drive reverse
    std::cout << "reverse mode\n";
    velocity = MAX_RAW_VELOCITY/3.0;
    sendWheelSkidCommands(-1*velocity, -1*velocity);
   } else if (msg->axes[DPAD_SIDE] > 0){  //drive left pointturn
    std::cout << "left mode\n";
    velocity = MAX_RAW_VELOCITY/3.0;
    sendWheelSkidCommands(-1*velocity, velocity);
   } else if (msg->axes[DPAD_SIDE] < 0){  //drive right pointturn
    std::cout << "right  mode\n";
    velocity = MAX_RAW_VELOCITY/3.0;
    sendWheelSkidCommands(velocity, -1*velocity);
   } else { // Stop motion
    std::cout << "stop mode\n";
    velocity = 0;
    sendWheelSkidCommands(velocity, velocity);
  }

}
*/

/**
 * Callback for driving commands with skid steer
 */
void skidCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  std::cout << "Hebi Skid Drive Command: Velocity x/y=" << msg->linear.x << " / " << msg->linear.y << ".\n";
  sendWheelSkidCommands(msg->linear.x, msg->linear.y);
}


/**
 * Callback for driving commands with mecanum omni wheel drive
 */
// void omniCallback(const geometry_msgs::Twist::ConstPtr& msg)
// {
//   std::cout << "Hebi Omni Drive Command: Velocity=" << msg->linear.x << " Angle=" << msg->angular.x << ".\n";
//   sendWheelTranslationCommands(msg->linear.x, msg->angular.x);
// }

/**
 * callback for joint commands (ie low level command for each axis)
 */
void commandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // TODO: check names instead of just length?
  if ((group_g->size() != msg->position.size() && msg->position.size() != 0) ||
      (group_g->size() != msg->velocity.size() && msg->velocity.size() != 0) ||
      (group_g->size() != msg->effort.size() && msg->effort.size() != 0))
  {
    ROS_INFO("Command length did not match group size.");
    std::cerr << "Command length did not match group size.";
    return;
  }
  hebi::GroupCommand cmd(group_g->size());
  for (int i = 0; i < msg->position.size(); i++)
    cmd[i].actuatorCommand().setPosition(msg->position[i]);
  for (int i = 0; i < msg->velocity.size(); i++)
    cmd[i].actuatorCommand().setVelocity(msg->velocity[i]);
  for (int i = 0; i < msg->effort.size(); i++)
    cmd[i].actuatorCommand().setTorque(msg->effort[i]);
  group_g->sendCommand(cmd);
}

/**
 * This node publishes feedback from named joints in the URDF model on the
 * parameter server under "robot_description".
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "stalker_interface");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Get the names of non-fixed joints in the model:
  std::vector<std::string> joint_names = {"Front", "Rear", "Front", "Rear"};
  std::vector<std::string> family_names = {"Drive_Left", "Drive_Left", "Drive_Right", "Drive_Right"};

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  // Get the lookup and wait to populate (TODO: check for null?)
  hebi::Lookup lookup;
  sleep(2);
  lookup.printTable();

  // Get the group
  for (int i = 0; i < joint_names.size(); i++)
  {
    std::cout << "looking for: " << std::endl;
    std::cout << joint_names[i] << std::endl;
    std::cout << family_names[i] << std::endl;
  }
  std::unique_ptr<hebi::Group> group(lookup.getGroupFromNames(joint_names, family_names, 1000));
  if (!group)
  {
    ROS_INFO("Could not find modules on network! Quitting!");
    return -1;
  }
  // THIS IS A HACK to get around limited callback options for ROS subscribe call and the lack of a class for this node.
  group_g = group.get();

  std::cout << "Found modules!" << std::endl;
  ROS_INFO("Found modules!");

  // declare odometry message, required by robot state publisher.

  // Add an async feedback handler, sending feedback from one module to control a second
  group->addFeedbackHandler(
    [&joint_pub, &joint_names](hebi::GroupFeedback* const fbk)->void
      {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        // Add feedback:
        for (int i = 0; i < joint_names.size(); i++)
        {
          msg.name.push_back(joint_names[i].c_str());
          if ((*fbk)[i].actuatorFeedback().hasPosition())
            msg.position.push_back((*fbk)[i].actuatorFeedback().getPosition());
          if ((*fbk)[i].actuatorFeedback().hasVelocity())
            msg.velocity.push_back((*fbk)[i].actuatorFeedback().getVelocity());
          if ((*fbk)[i].actuatorFeedback().hasTorque())
            msg.effort.push_back((*fbk)[i].actuatorFeedback().getTorque());
        }

        joint_pub.publish(msg);
      });
  std::cout << "Added handler!" << std::endl;
  // Set the rate at which this gets called.  (TODO: make this a parameter)
  group->setFeedbackFrequencyHz(25);
  std::cout << "Set handler frequency!" << std::endl;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub_joint_command = n.subscribe("joint_commands", 1, commandCallback);
  ros::Subscriber sub_skid_vel = n.subscribe("cmd_skid_drive", 1, skidCallback);
  // ros::Subscriber sub_omni_vel = n.subscribe("cmd_omni_drive", 1, omniCallback);
  ros::Subscriber sub_cmd_joy = n.subscribe("joy", 1, joyCallback);

  while (ros::ok())
  {
    //sleep(1);
    //ros::spinOnce();
    ros::spin();
  }

  // Stop the async callback before returning and deleting objects.
  group->clearFeedbackHandlers();

  sleep(1); // prevent segfaults? (TODO: needed?)

  return 0;
}
