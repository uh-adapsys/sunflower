#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sf_lights_msgs/LightsAction.h>
#include <libphidgets/phidget21.h>
#include <exception>

//#define NOPHIDGET

class LightsAction {
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<sf_lights_msgs::LightsAction> as_;
	std::string action_name_;
	sf_lights_msgs::LightsFeedback feedback_;
	sf_lights_msgs::LightsResult result_;

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

	LightsAction(std::string name, std::string topic) :
			as_(nh_, topic, boost::bind(&LightsAction::executeCB, this, _1),
					false), action_name_(name) {
#ifndef NOPHIDGET
		CPhidget_enableLogging(PHIDGET_LOG_WARNING, NULL);
		CPhidgetInterfaceKit_create(&phidgetInterface); //create Phidget Handle
		CPhidget_open((CPhidgetHandle) phidgetInterface, -1); //open the port for connection to Phidget
		int resultDisplay = CPhidget_waitForAttachment(
				(CPhidgetHandle) phidgetInterface, 2000);
		if (resultDisplay > 0) {
			const char *err;
			CPhidget_getErrorDescription(resultDisplay, &err);
			ROS_ERROR("Failed to open Phidget Device, error: %s", err);
			return;
		} else {
			const char* name;

			CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_RED,
					0);
			CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_GREEN,
					0);
			CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_BLUE,
					0);
			CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_BOARD,
					0);
			CPhidget_getDeviceName((CPhidgetHandle) phidgetInterface, &name);
			ROS_INFO("Connected to Phidget: %s", name);
		}
#endif

		ROS_INFO("Started Lights server");
		as_.start();
	}

	~LightsAction(void) {
#ifndef NOPHIDGET
		CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_RED, 0);
		CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_GREEN, 0);
		CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_BLUE, 0);
		CPhidgetInterfaceKit_setOutputState(phidgetInterface, PHIDGET_BOARD, 0); //turn off the board
		CPhidget_disableLogging();
#endif
	}

	int handleError(int errorCode, const char* deviceName, int value) {
		// Everything is okay if errorCode = 0
		if (errorCode != 0) {
			const char *errorDescription;
			int err = CPhidget_getErrorDescription(errorCode,
					&errorDescription);
			if (err == 0) {
				ROS_ERROR("Settings state %i for %s failed, error: %s", value,
						deviceName, errorDescription);
			} else {
				ROS_ERROR("Settings state %i for %s failed", value, deviceName);
				ROS_ERROR(
						"An error occurred while retrieving error description.  Error code: %i",
						err);
			}
		}

		return 0;
	}

	void executeCB(const sf_lights_msgs::LightsGoalConstPtr &goal) {
		bool red = goal->rgb[0];
		bool green = goal->rgb[1];
		bool blue = goal->rgb[2];
		bool board = red || green || blue;

		ROS_INFO("%s: Setting lights: R(%i) G(%i) B(%i)", action_name_.c_str(),
				red, green, blue);

#ifndef NOPHIDGET
		int rErr = CPhidgetInterfaceKit_setOutputState(phidgetInterface,
				PHIDGET_RED, red);
		int gErr = CPhidgetInterfaceKit_setOutputState(phidgetInterface,
				PHIDGET_GREEN, green);
		int bErr = CPhidgetInterfaceKit_setOutputState(phidgetInterface,
				PHIDGET_BLUE, blue);
		int oErr = CPhidgetInterfaceKit_setOutputState(phidgetInterface,
				PHIDGET_BOARD, board);
#endif

		result_.result = rErr + gErr + bErr + oErr;

		if (result_.result == 0) {
			as_.setSucceeded(result_);
		} else {
			handleError(oErr, "Board", board);
			handleError(rErr, "Red", red);
			handleError(gErr, "Green", green);
			handleError(bErr, "Blue", blue);
			as_.setAborted(result_);
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "phidget_leds");

	std::string topic;
	if (ros::param::get("~topic", topic)) {
		LightsAction lights(ros::this_node::getName(), topic);
	} else {
		LightsAction lights(ros::this_node::getName(),
				ros::this_node::getName());
	}

	ros::spin();

	return 0;
}

