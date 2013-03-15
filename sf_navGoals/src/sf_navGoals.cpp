#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "SamClass.h"
#include <boost/thread.hpp>


//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseStamped.h>

/*
typedef actionlib::SimpleActionClient<eskorta_move::move_baseAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
           const move_baseResultConstPtr& result)
{
 ROS_INFO("Finished in state [%s]", state.toString().c_str());
 ROS_INFO("[X]:%f [Y]:%f [W]: %f",result->base_position.pose.position.x,
result->base_position.pose.position.y,
result->base_position.pose.orientation.w);
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
 ROS_INFO("[X]:%f [Y]:%f [W]: %f",feedback->base_position.pose.position.x,
feedback->base_position.pose.position.y,
feedback->base_position.pose.orientation.w);
}
*/

/*

void spinThread() //dealing with 
{
	int wait=0;

	while (1)
	{	
		//samgar set goal ?
		if (wait==100) {
			newGoalRequested=1;
			ROS_INFO("Sending goal" );
		}
		else ROS_INFO("NOT  Sending goal" );

		wait++;
	}	
} */

// This class just sends out some data 


int newGoalRequested=0;
using namespace yarp::os;

class SAMROS_sf_navGoals: public SamClass 
{
  private:

	BufferedPort<Bottle> world_RobotLocalisedPosePortOut; //current localised robot pose

	BufferedPort<Bottle> world_GoalPosePortIn; //goal (in map coordinate) to be reached by robot  
	BufferedPort<Bottle> world_GoalPoseFeedbackPortOut; //provide reaching target feedback

	BufferedPort<Bottle> base_GoalPosePortIn; //goal (robot ego coordinate) to be reached by robot  
	BufferedPort<Bottle> base_GoalPoseFeedbackPortOut; //provide reaching target feedback
  
	double x, y, th; // receiving goal from port

	int goalFeedbackPort; //to set the right feedback port to publish
	

  public:
	SAMROS_sf_navGoals(std::string name);
	virtual void SamInit();
	virtual void SamIter(){}
	void SamIterIn(move_base_msgs::MoveBaseGoal *goal);
	void SamIterOut(int goalState);
	void SamIterRobotLocalisedPoseOut(double robX, double robY, double robTh)
	{
		Bottle& bottleWorld_RobotLocalisedPosePortOut = world_RobotLocalisedPosePortOut.prepare();	  // prepare the bottle/port
		bottleWorld_RobotLocalisedPosePortOut.clear();
		bottleWorld_RobotLocalisedPosePortOut.addDouble(robX); //feedback
		bottleWorld_RobotLocalisedPosePortOut.addDouble(robY); //feedback
		bottleWorld_RobotLocalisedPosePortOut.addDouble(robTh); //feedback
		world_RobotLocalisedPosePortOut.write();
	}
};

SAMROS_sf_navGoals::SAMROS_sf_navGoals(std::string name): SamClass(name)
{
	goalFeedbackPort = 0;
}
 
void SAMROS_sf_navGoals::SamInit(void)
{
	newPort(&world_RobotLocalisedPosePortOut, "worldRobPose"); //current localised robot pose (in map coordinate)

	newPort(&world_GoalPosePortIn, "worldGoalPoseIn"); //goal (in map coordinate) to be reached by robot  
	newPort(&world_GoalPoseFeedbackPortOut, "worldGoalPoseFbOut"); //provide reaching target feedback

	newPort(&base_GoalPosePortIn, "baseGoalPoseIn"); //goal (robot ego coordinate) to be reached by robot  
	newPort(&base_GoalPoseFeedbackPortOut, "baseGoalPoseFbOut"); //provide reaching target feedback

	StartModule();	
	puts("started writer");
}

void SAMROS_sf_navGoals::SamIterOut(int goalState)
{
// send the right feedback to the right port
	if(goalFeedbackPort == 1){ 
		Bottle& bottleWorld_GoalPoseFeedbackPortOut = world_GoalPoseFeedbackPortOut.prepare();	  // prepare the bottle/port
		bottleWorld_GoalPoseFeedbackPortOut.clear();
		bottleWorld_GoalPoseFeedbackPortOut.addInt(goalState); //feedback
		world_GoalPoseFeedbackPortOut.write(true);
		goalFeedbackPort = 0; //stop sending msg
		ROS_INFO("Sending confirmation back to STEVE--------- %d", goalState);

	}

	if(goalFeedbackPort == 2){
		Bottle& bottleBase_GoalPoseFeedbackPortOut = base_GoalPoseFeedbackPortOut.prepare();	  // prepare the bottle/port
		bottleBase_GoalPoseFeedbackPortOut.clear();
		bottleBase_GoalPoseFeedbackPortOut.addInt(goalState); //feedback
		base_GoalPoseFeedbackPortOut.write(true);
		goalFeedbackPort = 0; //stop sending msg
		ROS_INFO("Sending confirmation back to REZA---------- %d", goalState);
	}
}
   
void SAMROS_sf_navGoals::SamIterIn(move_base_msgs::MoveBaseGoal *goal)
{
	//world Goal Pose
	Bottle *bottleWorld_GoalPosePortIn = world_GoalPosePortIn.read(false);
	if(bottleWorld_GoalPosePortIn != NULL){
		x  = bottleWorld_GoalPosePortIn->get(0).asDouble(); // x in m
		y  = bottleWorld_GoalPosePortIn->get(1).asDouble(); // y in m
		th = bottleWorld_GoalPosePortIn->get(2).asDouble(); // th in degree, Counter Clockwise +, Clockwise -

		goal->target_pose.header.frame_id = "map";//"base_footprint"; //"base_link";
		goal->target_pose.header.stamp = ros::Time::now();
		goal->target_pose.pose.position.x = x; //x in m 
		goal->target_pose.pose.position.y = y; //y in m if in mm need to m by i.e. x*1e-3 
		goal->target_pose.pose.orientation= tf::createQuaternionMsgFromYaw(th*M_PI/180); //convert degree to rad		
		newGoalRequested = 1;
		goalFeedbackPort = 1; //set the world output port to publish goal result
	}
	else {
		//base Goal Pose
		Bottle *bottleBase_GoalPosePortIn = base_GoalPosePortIn.read(false);
		if(bottleBase_GoalPosePortIn != NULL){
			x  = bottleBase_GoalPosePortIn->get(0).asDouble(); // x in m
			y  = bottleBase_GoalPosePortIn->get(1).asDouble(); // y in m
			th = bottleBase_GoalPosePortIn->get(2).asDouble(); // th in degree, Counter Clockwise +, Clockwise -

			goal->target_pose.header.frame_id = "base_footprint"; //"base_link";
			goal->target_pose.header.stamp = ros::Time::now();
			goal->target_pose.pose.position.x = x; //x in m
			goal->target_pose.pose.position.y = y; //y in m if in mm need to m by i.e. x*1e-3 
			goal->target_pose.pose.orientation= tf::createQuaternionMsgFromYaw(th*M_PI/180); //convert degree to rad
			newGoalRequested = 1;
			goalFeedbackPort = 2; //set the base output port to publish goal result
		} 
	}
}

SAMROS_sf_navGoals samgar_sf_navGoals("navGoals");

void amcl_localisation_poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_estimate)
{
//double x, y, th;
	//x=pose_estimate->pose.pose.position.x;
	//y=pose_estimate->pose.pose.position.y;
	//th=(180/M_PI)*tf::getYaw(pose_estimate->pose.pose.orientation);
	//printf("new pose [%0.4f,%0.4f,%0.2f]", pose_estimate->pose.pose.position.x, pose_estimate->pose.pose.position.y,(180/M_PI)*tf::getYaw(pose_estimate->pose.pose.orientation));

samgar_sf_navGoals.SamIterRobotLocalisedPoseOut(pose_estimate->pose.pose.position.x, pose_estimate->pose.pose.position.y, (180/M_PI)*tf::getYaw(pose_estimate->pose.pose.orientation));


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sf_navGoals");
	
	ros::NodeHandle n;

	ros::Subscriber amcl_localisation_pose = n.subscribe <geometry_msgs::PoseWithCovarianceStamped> ("/amcl_pose", 1, amcl_localisation_poseCB);

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	//tell the action client that we want to spin a thread by default
  	MoveBaseClient ac("move_base", true);
  
  	move_base_msgs::MoveBaseGoal goal;

	

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	//boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "map";//"base_footprint"; //"base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 0.0;
	goal.target_pose.pose.position.y = 0.0;
	goal.target_pose.pose.orientation= tf::createQuaternionMsgFromYaw(1.0*M_PI/180);
    	
	newGoalRequested=0;
 
	ros::Rate r(10.0);
	
	samgar_sf_navGoals.SamInit();

	while (n.ok())
	{	
		samgar_sf_navGoals.SamIterIn(&goal); //check if we have any new goal request

		if(newGoalRequested)
		{	ac.sendGoal(goal);
			newGoalRequested=0;
  			ROS_INFO("waiting for goal result" );
			ac.waitForResult();
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    			ROS_INFO("Hooray, sunflower reached the goal");
				samgar_sf_navGoals.SamIterOut(1); //done
			}
  			else {
    			ROS_INFO("sunflower failed to reach goal some reason");		
				samgar_sf_navGoals.SamIterOut(0); //failed
			}
		/*
		 	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("Hooray, the base moved to goal");
		  	else 
		  	if(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
				ROS_INFO("The base failed to move: ACTIVE");
		  	else 
		  	if(ac.getState() == actionlib::SimpleClientGoalState::RECALLED)
				ROS_INFO("The base failed to move: RECALLED");
		  	else 
		  	if(ac.getState() == actionlib::SimpleClientGoalState::REJECTED)
				ROS_INFO("The base failed to move: REJECTED");
		  	else 
		  	if(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
				ROS_INFO("The base failed to move: PREEMPTED");
		  	else 
		  	if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
				ROS_INFO("The base failed to move: ABORTED");
		  	else 
		  	if(ac.getState() == actionlib::SimpleClientGoalState::LOST)
				ROS_INFO("The base failed to move: LOST");
		*/
		}
  		ros::spinOnce();
		r.sleep();
	}

  return 0;
}



