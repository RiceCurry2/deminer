/* Bondary definitions and working on some sounds - There will be pushed to this file through the weekend */
#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <sstream>
#include <vector>
#include "std_msgs/String.h"

#include "sound_play/sound_play.h"

#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>


/*********************************************************************************************
 * @brief           Standard definitions
 * @param _point    Namespace: geometry_msgs::PointStamped
 * @param _poly     Namespace: geometry_msgs::PolygonStamped
 *********************************************************************************************/
geometry_msgs::PolygonStamped _poly;
geometry_msgs::PointStamped _point;

static ros::Publisher pointline_pub;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::string path_to_sounds = "../catkin_ws/src/deminer/sounds/";
int cb = 0;
int z = 0; /* Not used ATM */ 
bool awaiting_center(false);

/*********************************************************************************************
 * @brief Console function and debugging tools for ease of debugging
 * LOG() must stay after completion of code
 * 
 *********************************************************************************************/
void LOG(const char* message){
    std::cout << message << std::endl;
}

/*********************************************************************************************
 * msgCallback function for printing the clicked coordinate in terminal window
 * The '&' represents that the PointStamped publisher are passed by reference from the main.
 * This is done for the purpose of using the 'msg' nameclasses outside the main loop.
 *********************************************************************************************/
void msgCallback(const geometry_msgs::PointStamped::ConstPtr& cpoint){
    /****** Print point no. 'x' to console ******/
    std::cout<<"This was point "<<z<<".\n";

    /****** ROS_INFO the clicked point ******/
    ROS_INFO("Clicked point is: \x1B[92m%f\033[0m, \x1B[94m%f\033[0m\n\n" , cpoint->point.x, cpoint->point.y);

    /****** Add 1 to 'x' to tell which point was marked ******/
    z++; 
}

void pointClick(const geometry_msgs::PointStamped& cpoint){
    float x_point = cpoint.point.x;
    float y_point = cpoint.point.y;
    
    if (cb >= 4){
    std::cout <<"5th point was added, deminer will start from (x,y): x:" << x_point << "," << "y:" << y_point << std::endl;
    /* Here goes the function to send_goal */ 
    cb = 0;
    }
        else if (cb <= 3){
        std::cout << "Adding point to database:" << std::endl;
        cb++;
    }
    
    return;
}

/* Just a test lolz */


void pathPlanner(){
    return;
};

/*********************************************************************************************
 * This is a function for the purpose of playing sound 
 * The '&' represents that the sound client publisher are passed by reference from the main.
 * This is done for the purpose of using the 'sc.' nameclasses outside the main loop.
 *********************************************************************************************/
void playSound(int a, sound_play::SoundClient& sc){

    /****** Defining the path to our sound files ******/
    path_to_sounds = "../catkin_ws/src/deminer/sounds/";

    /****** Switch statement to handle the sound files ******/
    switch(a)
    {
    case 1:
        sc.playWave(path_to_sounds+"boundary_selection.wav");
        return;
    case 2:
        sc.playWave(path_to_sounds+"boundary_selection.wav");
        return;
    case 3:
        sc.playWave(path_to_sounds+"boundary_selection.wav");
        return;
    default:
        LOG("Error: The playsound function recieved an undefined number");
    }

    return;
}

        void boundingBox(const geometry_msgs::PoseArrayConstPtr& msg)
        {
        const std::vector<geometry_msgs::Pose>& ps = msg->poses;
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/deminer";

            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "boundingBox";
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = 0; //tag id
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;  //tag type
            line_strip.scale.x = 0.05;
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;

            geometry_msgs::Point p;
            p.x = ps[0].position.x;
            p.y = ps[0].position.y;
            p.z = ps[0].position.z;
            line_strip.points.push_back(p);
        }

/*********************************************************************************************
 * The master main file
 * Handles ROS init, publish/subscribe and functionality (via functions)
 *
 *********************************************************************************************/

int main(int argc, char **argv){

    /****** Initialising this ROS node ******/
    ros::init(argc, argv, "click_point");

    /****** Initialising the nodehandle for publish/subsribe ******/
    ros::NodeHandle n;

    /****** Initialising the sound client ******/
    sound_play::SoundClient sc;

    /****** Initialising the sound client ******/
    ros::spinOnce();

    /****** Initialising the sound client ******/
    LOG("Please wait.. Initialising click_point node \n. \n. \n.\n");

    /****** A sleep needed for initialisation (SoundClient) ******/
    ros::Duration(2).sleep();
    
    /****** Calling the playSound function with a sound value and a reference to the SoundClient ******/
    playSound(1, sc);

    std::cout<<"click_point node initialised \n. \n. \n.\n"<<std::endl;

    std::cout<<"\e[1mCOORDINATE FINDER\e[0m \nUsage: \n1:Click the 'publish a point' tool in rVIZ. \n2:Click within the map boundary \n3:Retrieve the coordinate in this terminal windows \n\n-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴-̴"<<std::endl;

    _poly.header.frame_id = "map";

    ros::Subscriber sub = n.subscribe("/clicked_point", 1000, &pointClick);
    
    //pointline_pub = n.advertise<visualization_msgs::Marker>("demining_polygon_marker", 10, pointLines);
    
    //poly_sub = n.subscribe<geometry_msgs::PolygonStamped>("polygon_point", 10, polyReciever);

    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseArray>("/whycon/poses", 10, &boundingBox);

    ROS_INFO("Please use the 'Point' tool in Rviz to select an exporation boundary.");

    //ros::WallTimer wall_timer = n.createWallTimer(ros::WallDuration(0.1), &pointLines);

    ros::spin();

    return 0;
}

/* 
   int main( int argc, char** argv )
   {
     ros::init(argc, argv, "points_and_lines");
     ros::NodeHandle n;
     ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
   
     ros::Rate r(30);
   
     float f = 0.0;
     while (ros::ok())
     {
   
       visualization_msgs::Marker points, line_strip;
       points.header.frame_id = line_strip.header.frame_id = "/map";
       points.header.stamp = line_strip.header.stamp = ros::Time::now();
       points.ns = line_strip.ns = "points_and_lines";
       points.action = line_strip.action = visualization_msgs::Marker::ADD;
       points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
   
   
   
       points.id = 0;
       line_strip.id = 1;
       
   
       points.type = visualization_msgs::Marker::POINTS;
       line_strip.type = visualization_msgs::Marker::LINE_STRIP;
   
       points.scale.x = 0.2;
       points.scale.y = 0.2;
  
       line_strip.scale.x = 0.1;
   
   
       points.color.g = 1.0f;
       points.color.a = 1.0;
   
       line_strip.color.b = 1.0;
       line_strip.color.a = 1.0;
  
   for (uint32_t i = 0; i < 1; ++i)
       {
        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
  
        points.points.push_back(p);
        line_strip.points.push_back(p);
   
       }
   for (uint32_t i = 0; i < 1; ++i)
       {
        geometry_msgs::Point p;
        p.x = -1.1;
        p.y = -5.5;
        p.z = 0;
  
        points.points.push_back(p);
        line_strip.points.push_back(p);
   
       }
   for (uint32_t i = 0; i < 1; ++i)
       {
        geometry_msgs::Point p;
        p.x = -3.4;
        p.y = -5.1;
        p.z = 0;
  
        points.points.push_back(p);
        line_strip.points.push_back(p);
   
       }
   for (uint32_t i = 0; i < 1; ++i)
       {
        geometry_msgs::Point p;
        p.x = 1.0;
        p.y = 4.4;
        p.z = 0;
  
        points.points.push_back(p);
        line_strip.points.push_back(p);
   
       }
   for (uint32_t i = 0; i < 1; ++i)
       {
        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
  
        points.points.push_back(p);
        line_strip.points.push_back(p);
   
       }


      marker_pub.publish(points);
      marker_pub.publish(line_strip);
  
      r.sleep();
  
      f += 0.04;
    }
  }
  */