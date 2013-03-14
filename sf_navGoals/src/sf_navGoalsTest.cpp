#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


/// sf_navGoalsTest.cpp is for testing CB functions through actionlib -
//	phase 0 - done - all the pointers are working and bug free
//  phase 1 - 

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
ROS_INFO("[X]:%f [Y]:%f [W]: %f",feedback->base_position.pose.position.x, feedback->base_position.pose.position.y, feedback->base_position.pose.orientation.w);
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_eskorta");
	Client client("eskorta_base", true);


   	while(!client.waitForServer(ros::Duration(5.0))){
   		ROS_INFO("Waiting for the move_base action server to come up");
  	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = -0.423;
	goal.target_pose.pose.position.y = 1.915;
	goal.target_pose.pose.orientation.w = 1.000;
	
	ROS_INFO("Sending goal");
	client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	client.waitForResult();

 	ros::spin();

return 0;
} 




/*
// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const move_baseResultConstPtr& result)
{
 ROS_INFO("Finished in state [%s]", state.toString().c_str());
 ROS_INFO("[X]:%f [Y]:%f [W]: %f",result->base_position.pose.position.x, result->base_position.pose.position.y, result->base_position.pose.orientation.w);
 ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
 ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_baseFeedbackConstPtr& feedback)
{
 ROS_INFO("[X]:%f [Y]:%f [W]: %f",feedback->base_position.pose.position.x,feedback->base_position.pose.position.y, feedback->base_position.pose.orientation.w);
}


int newGoalRequested=0;

// This class just sends out some data 

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sf_navGoals");
  
	Client ac("move_base", true);
  	move_base_msgs::MoveBaseGoal goal;

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	
  	ros::spin();

  return 0;
}
*/


