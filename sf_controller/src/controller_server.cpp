#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sf_controller/ControllerAction.h>
#include <phidget21.h>
#include <exception>

class ControllerAction {
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<sf_controller::ControllerAction> as_;
	std::string action_name_;
	sf_controller::ControllerFeedback feedback_;
	sf_controller::ControllerResult result_;

	static const int PHIDGET_BOARD = 0;
	static const int PHIDGET_RED = 1;
	static const int PHIDGET_BLUE = 2;
	static const int PHIDGET_GREEN = 3;
	static const int PHIDGET_LED_OFF = 0;
	static const int PHIDGET_LED_ON = 1;

public:

	ControllerAction(std::string name) :
			as_(nh_, name, boost::bind(&ControllerAction::executeCB, this, _1),
					false), action_name_(name) {

		ROS_INFO("Started Controller server");
		as_.start();
	}

	~ControllerAction(void) {
	}

	void executeCB(const sf_controller::ControllerGoalConstPtr &goal) {
		bool success = true;

		ROS_INFO("%s: Settings %s to %s", action_name_.c_str(), goal->component.c_str(), goal->postion.c_str());

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
	ControllerAction lights(ros::this_node::getName());
	ros::spin();

	return 0;
}

