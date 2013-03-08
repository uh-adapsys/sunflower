#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sf_lights/LightsAction.h>
#include <phidget21.h>
#include <exception>

//#define NOPHIDGET

class LightsAction {
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<sf_lights::LightsAction> as_;
	std::string action_name_;
	sf_lights::LightsFeedback feedback_;
	sf_lights::LightsResult result_;

#ifndef NOPHIDGET
	CPhidgetInterfaceKitHandle phidgetInterface;
#endif

	static const int PHIDGET_BOARD = 0;
	static const int PHIDGET_RED = 1;
	static const int PHIDGET_BLUE = 2;
	static const int PHIDGET_GREEN = 3;
	static const int PHIDGET_LED_OFF = 0;
	static const int PHIDGET_LED_ON = 1;

public:

	LightsAction(std::string name) :
			as_(nh_, name, boost::bind(&LightsAction::executeCB, this, _1),
					false), action_name_(name) {
#ifndef NOPHIDGET
		CPhidgetInterfaceKit_create(&phidgetInterface); //create Phidget Handle
		CPhidget_open((CPhidgetHandle) phidgetInterface, -1); //open the port for connection to Phidget
		int resultDisplay = CPhidget_waitForAttachment(
				(CPhidgetHandle) phidgetInterface, 2000);
		if (resultDisplay > 0) {
			const char *err;
			char buf[1000];

			CPhidget_getErrorDescription(resultDisplay, &err);
			sprintf(buf, "error: %s\n", err);
			ROS_INFO("Failed to open Phidget Device, error: %s", buf);
			return;
		} else {
			const char* name;

			CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_RED, 0);
			CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_GREEN, 0);
			CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_BLUE, 0);
			CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_BOARD, 1); //turn on the board#
			CPhidget_getDeviceName((CPhidgetHandle) phidgetInterface, &name);
			ROS_INFO("Connected to Phidget: %s", name);
		}
#endif


		ROS_INFO("Started Lights server");
		as_.start();
	}

	~LightsAction(void) {
	}

	void executeCB(const sf_lights::LightsGoalConstPtr &goal) {
		bool success = true;

		int red = goal->rgb[0];
		int green = goal->rgb[1];
		int blue = goal->rgb[2];

		ROS_INFO("%s: Settings lights: R(%i) G(%i) B(%i)", action_name_.c_str(),
				red, green, blue);

#ifndef NOPHIDGET
		CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_RED, red);
		CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_GREEN,
				green);
		CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_BLUE,
				blue);

#endif

		result_.result = 0;

		if (success) {
			as_.setSucceeded(result_);
		} else {
			as_.setAborted(result_);
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "lights");
	LightsAction lights(ros::this_node::getName());
	ros::spin();

	return 0;
}

