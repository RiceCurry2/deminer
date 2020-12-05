/* A wasted day of work, saving this for the purpose of reusing the simple_action_client */
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);

/** declare the coordinates of interest **/
double x01 = 1.791505;
double y01 = -0.596119;
double x2 = 0.181379;
double y2 = -0.598247;
double x3 = 0.181379;
double y3 = -0.397506;
double x4 = 1.798960;
double y4 = -0.395655;
double x5 = 1.798960;
double y5 = -0.192285;
double x6 = 0.192360;
double y6 = -0.201740;
double x7 = 0.192360;
double y7 = -0.201740;
double x8 = 0.192360;
double y8 = 0.006572;
double x9 = 1.801494;
double y9 = 0.004412;
double x10 = 1.801494;
double y10 = 0.199987;
double x11 = 1.801494;
double y11 = 0.194055;
double x12 = 0.191272;
double y12 = 0.406423;
double x13 = 0.191272;
double y13 = 0.407750;

bool goalReached = false;

double count = 0;
int driving = 0;

int main(int argc, char** argv){
	ros::init(argc, argv, "coord_run");
	ros::NodeHandle n;
	ros::spinOnce();
		if (driving == 0){
		if (count == 0){
			driving = 1;
			goalReached = moveToGoal(x01, y01);
		}else if (count == 1){
			driving = 1;
			goalReached = moveToGoal(x2, y2);
		}else if (count == 2){
			driving = 1;			
			goalReached = moveToGoal(x3, y3);
		}else if (count == 3){
			driving = 1;
			goalReached = moveToGoal(x4, y4);
		}else if (count == 4){
			driving = 1;
			goalReached = moveToGoal(x5, y5);
		}else if (count == 5){
			driving = 1;
			goalReached = moveToGoal(x6, y6);
		}else if (count == 6){
			driving = 1;
			goalReached = moveToGoal(x7, y7);
		}else if (count == 7){
			driving = 1;
			goalReached = moveToGoal(x8, y8);
		}else if (count == 8){
			driving = 1;
			goalReached = moveToGoal(x9, y9);
		}else if (count == 9){
			driving = 1;
			goalReached = moveToGoal(x10, y10);
		}else if (count == 10){
			driving = 1;
			goalReached = moveToGoal(x11, y11);
		}else if (count == 11){
			driving = 1;
			goalReached = moveToGoal(x12, y12);
		}else if (count == 12){
			driving = 1;
			goalReached = moveToGoal(x13, y13);
			count = 1;
	}
	}

		if (driving!=0){
			if (goalReached){
				ROS_INFO("Congratulations!");
				driving = 0;
				count++;
				ros::spinOnce();
			}else{
				ROS_INFO("Hard Luck!");
			}
		}
	while(driving !=1);
	return 0;
}

bool moveToGoal(double xGoal, double yGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");

		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

} 