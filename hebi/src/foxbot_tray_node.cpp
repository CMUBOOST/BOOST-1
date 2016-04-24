/*
 *Controller for sending tray commands to hebi controllers
 * Author: David Kohanbash 2015
 *
 *Send sample command: topic pub /cmd_tray geometry_msgs/Vector3 "  Then press tab
 *
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Vector3.h> 
#include <urdf/model.h>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

#include <sstream>
#include <string>
#include <vector>
#include <math.h>

// Array index of each axis to send to controllers
const int LIFT_AXIS = 0;
const int TRAY_EXTEND_AXIS = 1;
const int TRAY_FINGER_AXIS = 2;

// TORQUE signs are to travel up/forward
const double LIFT_AXIS_TORQUE = 0.35;
const double TRAY_EXTEND_AXIS_TORQUE = -0.1;
const double TRAY_FINGER_AXIS_TORQUE = 0.35;

// How close do we need the position to be
const double LIFT_AXIS_POSITION_MARGIN = 0.015;
const double TRAY_EXTEND_AXIS_POSITION_MARGIN = 0.02;
const double TRAY_FINGER_AXIS_POSITION_MARGIN = 0.015;
 
// Values in both lines below are updated regularaly... No need to udpate them manualy.
volatile double liftPositionCurrent = 0, extendPositionCurrent = 0, extendFingerPositionCurrent = 0;
volatile float analog_pin_1_module_1 = 0, analog_pin_1_module_2 = 0, analog_pin_1_module_3 = 0;

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
 * Retrieves the current value of the analog IO pin (0-5V). Returns 0 if no value is found.
 * The pin number is assumed to be 1-based (e.g., matching the diagram)
 */
/*float getIoFbk(int module, int pinNumber)
{
  TODO: SYNCHRONOUS GROUP FEEDBACK NOT YET FINISHED!
  hebi::
}*/

// Verify limits (in meters) of desired position are within user defined ranges.
// The convert* functions below check if current position is valid (only hits if a real problem)  
int checkDesiredPositionLimits(int axis, double desiredPosition)
{
	switch(axis) {
		case LIFT_AXIS:
			if (desiredPosition > 0.9) return(-1);
			if (desiredPosition < 0.7) return(-1);
			break;
		case TRAY_EXTEND_AXIS:
			if (desiredPosition > 0.2) return(-1);
			if (desiredPosition < -0.26) return(-1);
			break;
		case TRAY_FINGER_AXIS:
			if (desiredPosition > 0.11) return(-1);
			if (desiredPosition < 0.03) return(-1);
			break;
		default:
			return(-1);
	}

	return(1);
}

// Convert the raw ADC string pot value to height of the bottom of the tray mechanism to the ground.
// Height limits are really 0.692 - 0.908m
double convertLiftPot2Height(double pot) 
{
	double height = (-0.1356 * pot) + 1.2078;
	if ((height > 0.91) || (height < 0.68)){
		std::cout << "\tERROR: Lift hardware limit reached! (" << height << ")\n";
		return 999;
	} else {
		return height;
	}
}

// Convert the raw ADC string pot value of tray relative to the edge of mecahnism. + is extended. - is pulled in.
// Extension limits are really 0.215 - -0.264m
double convertTrayPot2Extension(double pot) 
{
	double extend = (-0.1384 * pot) + 0.4179;
	if ((extend > 0.22) || (extend < -0.27)){
		std::cout << "\tERROR: Tray hardware limit reached! (" << extend << ")\n";
		return 999;
	} else {
		return extend;
	}
}

// Convert the raw ADC pot value from fully retracted to fuly extended.
// The max retracted position is 0.025m (NOT 0)
// Extension limits are really 0.025 - 0.116m
double convertFingerPot2Extension(double pot) 
{
	double extend = (-0.0213 * pot) + 0.1255;

	if ((extend > 0.117) || (extend < 0.024)){
		std::cout << "\tERROR: Finger extend hardware limit reached! (" << extend << ")\n";
		return 999;
	} else {
		return extend;
	}
}

// Take an axis and then get us to that position using the string pots position
// String pots are updated asyncronously
int positionController (int axis, double desiredPosition, double torqueCmd)
{
  if ((axis < 0) || (axis > 2)) {
	std::cout << "\tERROR: Axis:" << axis << " does not exist\n";
	return(-1);
  }

  // Check if the command is within the user defined acceptable range
  int isPositionValid = checkDesiredPositionLimits(axis,desiredPosition);
  if (isPositionValid < 0) {
	std::cout << "\t LIMIT WARNING: Axis:" << axis << " commanded position out of range\n";
	return(-1);
  } else {
	std::cout << "\tMOTION START: Axis:" << axis << " desiredPosition:" << desiredPosition << " torque:" <<torqueCmd << "\n";	
  }

  // Initialize al of the values in the motor array. We want other axis to be 0 command
  hebi::GroupCommand cmd(3);
  cmd[LIFT_AXIS].actuatorCommand().setTorque(0);
  cmd[TRAY_EXTEND_AXIS].actuatorCommand().setTorque(0);
  cmd[TRAY_FINGER_AXIS].actuatorCommand().setTorque(0);

  int goal = 0;
  double currentPos;

  while (ros::ok() &&  (goal != 1)) {

	double positionMargin;
	
	switch(axis) {
		case LIFT_AXIS:
			currentPos = convertLiftPot2Height(analog_pin_1_module_1);
			positionMargin = LIFT_AXIS_POSITION_MARGIN;
			break;
		case TRAY_EXTEND_AXIS:
			currentPos = convertTrayPot2Extension(analog_pin_1_module_2);
			positionMargin = TRAY_EXTEND_AXIS_POSITION_MARGIN;
			break;
		case TRAY_FINGER_AXIS:
			currentPos = convertFingerPot2Extension(analog_pin_1_module_3);
			positionMargin = TRAY_FINGER_AXIS_POSITION_MARGIN;
			break;
		default:
			currentPos = 999; //ie bad value
	}
	
	//std::cout << "Axis:" << axis << " desiredPosition:" << desiredPosition << " currentPosition:" << currentPos << " torque:" <<torqueCmd << "\n";	

	if (currentPos == 999) return(-1);
				
	if ((currentPos < (desiredPosition + positionMargin)) && (currentPos > (desiredPosition - positionMargin))) {
  		cmd[axis].actuatorCommand().setTorque(0);
		goal = 1;
 	} else if(currentPos > desiredPosition) {
  		cmd[axis].actuatorCommand().setTorque(-1 * torqueCmd);
	} else if (currentPos < desiredPosition) {
  		cmd[axis].actuatorCommand().setTorque(torqueCmd);
	} else {
  		cmd[axis].actuatorCommand().setTorque(0);
		goal = 1;
	}
  	group_g->sendCommand(cmd);
  	usleep(50000);  // Experimental value for controlling rate of sending commands. 30000 works but I am being conservative
  }
  
  // Make sure that all motion is stopped
  // This should not be needed and can propably be deleted.
  //usleep(30000);
  //cmd[axis].actuatorCommand().setTorque(0.0);
  //group_g->sendCommand(cmd);
  
  std::cout << "\tMOTION COMPLETE: Axis:" << axis << " desiredPosition:" << desiredPosition << " currentPosition:" << currentPos << "\n";	

  return(1);
}



/**
 * Controller for moving the lift, tray extension, and tray finger motors. Motors are moved in that order.
 * The finger itself is controlled by 
 */
int sendTrayCommands(double liftPosition, double trayPosition, double fingerPosition)
{
  std::cout << "Tray Commanded Positions:" << liftPosition << " extend_position:" << trayPosition << " extend_finger_position:" << fingerPosition << "\n"; 
  
// Keep track of any errors in the motion commands
int status = 0;

if (positionController(LIFT_AXIS, liftPosition, LIFT_AXIS_TORQUE) == -1){
status +=1;
}
if (positionController(TRAY_EXTEND_AXIS, trayPosition, TRAY_EXTEND_AXIS_TORQUE) == -1){
status +=10;
}
if (positionController(TRAY_FINGER_AXIS, fingerPosition, TRAY_FINGER_AXIS_TORQUE) == -1){
status +=100;
}

  std::cout << "Done Status(" << status << ") \n";  

  return(status);
}

/**
 * Callback for joystick commands
 */
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  hebi::GroupCommand cmd(3);

  const int LEFT_TRIGGER_UP = 5;  //Button -  digital value 
  const int LEFT_TRIGGER_DOWN = 5;  // axis - analog value [1,-1]
  const int BUTTON_A = 0;  // Button - digital value
  const int BUTTON_B = 1;  // Button - digital value
  const int BUTTON_X = 2;  // Button - digital value
  const int BUTTON_Y = 3;  // Button - digital value

  // First handle lift motion
  if (msg->axes[LEFT_TRIGGER_DOWN] < 0){
  	cmd[LIFT_AXIS].actuatorCommand().setTorque( -1*LIFT_AXIS_TORQUE);
  } else if (msg->buttons[LEFT_TRIGGER_UP] == 1){
  	cmd[LIFT_AXIS].actuatorCommand().setTorque(LIFT_AXIS_TORQUE);
  } else {
  	cmd[LIFT_AXIS].actuatorCommand().setTorque(0);
  } 

  // Now handle tray extend motion (X = retract, B = extend)
  if (msg->buttons[BUTTON_X] == 1){
  	cmd[TRAY_EXTEND_AXIS].actuatorCommand().setTorque(-1*TRAY_EXTEND_AXIS_TORQUE);
  } else if (msg->buttons[BUTTON_B] == 1){
  	cmd[TRAY_EXTEND_AXIS].actuatorCommand().setTorque(TRAY_EXTEND_AXIS_TORQUE);
  } else {
  	cmd[TRAY_EXTEND_AXIS].actuatorCommand().setTorque(0);
  } 

  // Now handle the finger extend motion (A = retract, Y = extend)
  if (msg->buttons[BUTTON_A] == 1){
  	cmd[TRAY_FINGER_AXIS].actuatorCommand().setTorque(-1*TRAY_FINGER_AXIS_TORQUE);
  } else if (msg->buttons[BUTTON_Y] == 1){
  	cmd[TRAY_FINGER_AXIS].actuatorCommand().setTorque(TRAY_FINGER_AXIS_TORQUE);
  } else {
  	cmd[TRAY_FINGER_AXIS].actuatorCommand().setTorque(0);
  } 

  group_g->sendCommand(cmd);
}

/**
 */
void trayCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  //std::cout << "Hebi Tray Command: Velocity lift/extend/finger=" << msg->x << " / " << msg->y << " / " << msg->z<< ".\n";
  sendTrayCommands(msg->x, msg->y,msg->z);
}

/**
 */
void trayPositionCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	liftPositionCurrent = msg->position[0];
	extendPositionCurrent = msg->position[1]; 	
  	//std::cout << "Tray current_positions:" << liftPositionCurrent << " extend_position:" << extendPositionCurrent << "\n"; 
  	//std::cout << "Tray analog_positions:" << convertLiftPot2Height(analog_pin_1_module_1) << "," << convertTrayPot2Extension(analog_pin_1_module_2) << "," << convertFingerPot2Extension(analog_pin_1_module_3) << "\n"; 
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
  ros::init(argc, argv, "foxbot_tray_interface");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Get the names of non-fixed joints in the model:
  std::vector<std::string> joint_names = {"drive_05", "drive_06", "drive_07"};
  std::vector<std::string> family_names = {"fox_tray_motor", "fox_tray_motor", "fox_tray_motor"};

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
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("tray_states", 10);

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
        // Check for module 1:
        if ((*fbk)[0].ioFeedback().hasPin(0))
          analog_pin_1_module_1 = ((*fbk)[0].ioFeedback().getPin(0));
        // Check for module 2:
        if ((*fbk)[1].ioFeedback().hasPin(0))
          analog_pin_1_module_2 = ((*fbk)[1].ioFeedback().getPin(0));
        // Check for module 3:
        if ((*fbk)[2].ioFeedback().hasPin(0))
          analog_pin_1_module_3 = ((*fbk)[2].ioFeedback().getPin(0));

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
  ros::Subscriber sub_tray = n.subscribe("cmd_tray", 1, trayCallback);
  ros::Subscriber sub_cmd_joy = n.subscribe("joy", 1, joyCallback);
  ros::Subscriber sub_positio = n.subscribe("tray_states", 1, trayPositionCallback);

  // Set initial encoder zero position
  //liftPositionZero = 
  //extendPositionZero = 

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
