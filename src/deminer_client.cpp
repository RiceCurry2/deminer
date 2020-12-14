#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/wall_timer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "geometry_msgs/PointStamped.h" //Maybe not used
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Pose.h" //Maybe not used
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/Point.h"
#include <tf/transform_datatypes.h>
#include "deminer/geometry_tools.h"

#include <sstream>
#include <vector>
#include <math.h>

#include "std_msgs/String.h"

#include "sound_play/sound_play.h"

#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>

#include <algorithm>

namespace deminer{

class DeminerClient{

/*********************************************************************************************
 * @brief           Private global definitions
 * @param _point    Namespace: geometry_msgs::PointStamped
 * @param _poly     Namespace: geometry_msgs::PolygonStamped
 *********************************************************************************************/
private:
    // Defining handlers
    ros::NodeHandle _n;
    sound_play::SoundClient _sc;

    // Actionlib "move_base" definitons 
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_client;
    move_base_msgs::MoveBaseGoal goal;

    // Polygon, pose and point definitions
    geometry_msgs::PolygonStamped _poly;
    geometry_msgs::PoseWithCovarianceStamped _pose;
    geometry_msgs::PointStamped _point; //maybe not used
    geometry_msgs::Point32 _as; //maybe not used

    // ROS Publishers
    ros::Publisher pointline_pub;
    ros::Publisher goal_m_pub;
    ros::Publisher boundingbox_pub; //maybe not used

    // ROS Subscribers
    ros::Subscriber pointclick_sub;
    ros::Subscriber amcl_pose_sub;

    // ROS Walltimers
    ros::WallTimer pointline_timer;
    ros::WallTimer goal_timer;

    // Defining the path to sound strings (used in PlaySound method)
    std::string path_to_sounds = "../catkin_ws/src/deminer/sounds/";

    // Defining (client side) variables
    bool awaiting_center;
    bool sortingdone;
    float robot_width = 0.30f;

    // Defining (server side) variables 
    bool goalReached;
    bool rotationComplete;
    bool donewaypoint;

    // Defining rotation float
    float qYaw;
    float qX;
    float qY;
    float qZ;



    /*********************************************************************************************
     * @brief                   Defining of coor2d structure member used for vector items
     * @param distance_p1_p2    Calculates the distance from P1 to P2
     * @return                  Distance from P1 (x1, x1) to P2 (x2, y2)
     *********************************************************************************************/
    struct coor2d {
        float x1;
        float y1;
        float x2;
        float y2;

        float distance_p1_p2()
        {
        std::cout << x1 << "  " << x2<< "  " << y1 << "  " << y2 << std::endl;
        float result = sqrt(pow(x2 - x1, 2.0f) + pow(y2 - y1, 2.0f));
        return result;
        }
    };

    /*********************************************************************************************
     * @brief                   Probably not used (Subject for deletion)
     *********************************************************************************************/
    struct minmax {
        float min_x; 
        float min_y;
        float max_x;
        float max_y;

        void reset() 
        {
        min_x = 0.0f; 
        min_y = 0.0f;
        max_x = 0.0f;
        max_y = 0.0f;
        }
    };

    /*********************************************************************************************
     * @brief                   Defining of s_goal structure member used for vector items
     *                          Holds a simple x and y to be used in goal calculations
     *********************************************************************************************/
    struct s_goal {
        float x; 
        float y;
    };
    
    /*********************************************************************************************
     * @brief                   Defining of vectors
     *********************************************************************************************/
    // The vector which are going to hold the polygon points for further calculations
    std::vector<coor2d> vec;
    // The vector which are going to hold the top vector points of the polygon -> line: P0 to P1
    std::vector<s_goal> goal_vec_top;
    // The vector which are going to hold the top vector points of the polygon -> line: P2 to P3
    std::vector<s_goal> goal_vec_bot;
    // The final goal vector which are going to hold the sorted goals 
    std::vector<s_goal> goal_vec;


    /*********************************************************************************************
     * @brief                   Probably not used (Subject for deletion)
     *********************************************************************************************/
    struct compare_xy {
        bool operator ()(const coor2d& left, const coor2d& right) const {
        return (left.x1 < right.x1) || ((left.x1 == right.x1) && (left.y1 < right.y1));
    }
    };

    struct compare_yx {
        bool operator ()(const coor2d& left, const coor2d& right) const {
        return (left.y1 < right.y1) || ((left.y1 == right.y1) && (left.x1 < right.x1));
    }
    };

    /*********************************************************************************************
     * @brief                   This is a method for the purpose of playing sound 
     * @param playSound         Takes in an integer and SoundClient handler
     * @return                  Sound feedback execution happens before return
     *********************************************************************************************/
    void playSound(int a, sound_play::SoundClient& _sc){

    // Switch statement to handle the sound files
    switch(a)
    {
    case 0:
        _sc.playWave(path_to_sounds + "boundary_selection.wav");
        return;
    case 1:
        _sc.playWave(path_to_sounds + "last_point.wav");
        return;
    case 2:
        _sc.playWave(path_to_sounds + "goal_reached.wav");
        return;
    case 3:
        _sc.playWave(path_to_sounds + "goal_failed.wav");
        return;
    default:
        ROS_ERROR("Error: The playsound function recieved an undefined number");
    }
    return;
}


    /*********************************************************************************************
     * @brief                   Publish markers for visualization of points for the search polygon
     *                          This is interconnected in the initialization with boost::bind
     *                         
     *********************************************************************************************/
    void pointLines(){
    
    // Float definition
    float f = 0.0;

    // Type definition of points and line_strip
    visualization_msgs::Marker points, line_strip;

    // Header initialization and namespace definiton
    points.header = line_strip.header = _poly.header;
    points.ns = line_strip.ns = "boundary_points";

        // ID definition to avoid inconsistencies
        points.id = 0;
        line_strip.id = 1;

    // Type definition of each marker
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // if the ROS handled polygon vector is not empty (proceed)
    if(!_poly.polygon.points.empty()){
        
        // Use marker actions to add points and lines to rVIZ
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        // Scaling definitions
        points.scale.x = points.scale.y = 0.1;
        line_strip.scale.x = 0.05;

        // BOOST_FOREACH iterates each point from the ROS handled polygon vector and into a conversion vector
        // Meaning that the point32 type from geometry_msgs::PolygonStamped->polygon->points are converted
        // From a float32 to a float64 data type (Enables visualization markers to interact with the point)
        BOOST_FOREACH(geometry_msgs::Point32 point, _poly.polygon.points){
        line_strip.points.push_back(costmap_2d::toPoint(point));
        points.points.push_back(costmap_2d::toPoint(point));
        }

        if(awaiting_center){
            line_strip.points.push_back(costmap_2d::toPoint(_poly.polygon.points.front()));
            // RED Color (Points and lines)
            points.color.r = 1.0;
            points.color.a = 1.0;
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;

        }else{
            // GREEN Color (Points and lines)
            points.color.g = 1.0f;
            points.color.a = 1.0;
            line_strip.color.g = 1.0f;
            line_strip.color.a = 1.0;

        }
        }else{
            points.action = line_strip.action = visualization_msgs::Marker::DELETE;
        }
        pointline_pub.publish(points);
        pointline_pub.publish(line_strip);
    }

    /*********************************************************************************************
     * @brief                   moveToGoal method handles the action client side which sends move-
     *                          ment goal and rotation goal at the same time                      
     *********************************************************************************************/
    bool moveToGoal(float xGoal, float yGoal, float next_xGoal, float next_yGoal)
    {
        // Wait for the action server to come up
        while (!mb_client.waitForServer(ros::Duration(5.0)))
        {
        ROS_INFO("Waiting for the move_base action server to come up");
        }
        
        // Define parameters used in moveToGoal method
        double dx;                  // Distance x values
        double dy;                  // Distance y values
        double theta;               // Theta is the radian

        // Setup frame parameters
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

            // find out which turndirection is better
            // the bigger the angle, the bigger turn, - when clockwise
            //turn = atan2(sin(angle_to_goal-theta), cos(angle_to_goal-theta))

            //if abs(angle_to_goal - theta < 0.1){
            //move_forward = True
        
        // Rotation to goal, quaternion from yaw via radians (Euler angles).

        ros::spinOnce;               // Process callbacks
        ros::Duration(1).sleep();    // Sleep for 1hz

        double X = qX;               // Robot X postition
        double Y = qY;               // Robot Y postition
        double Yaw = qYaw;           // Robot Yaw

        dx = next_xGoal - xGoal;     // Distance goal to next goal in x
        dy = next_xGoal - yGoal;     // Distance goal to next goal in y
        theta = atan2(dy, dx);       // Calculate angle through distance from goal to next goal in x and y

        // Console output angle and radians
        std::cout << "Angle to turn: " << theta*180/M_PI << std::endl;             // Convert to degress for the console output  
        std::cout << "Rads: " << theta << std::endl;                               // Display radians

        // Moving towards the goal and rotating to oriention of next goal
        goal.target_pose.pose.position.x = xGoal;                                  // Set x pose position to current goal
        goal.target_pose.pose.position.y = yGoal;                                  // Set y pose position to current goal
        goal.target_pose.pose.position.z = 0.0;                                    // Set z pose position to 0
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta); // Set target pose to the quaternion oriented towards the goal
    
        ROS_INFO("Sending goal location ...");
        mb_client.sendGoal(goal);    // Send calculated goal to the actionserver

        mb_client.waitForResult();   // Wait for result from the actionserver

        // If statement to handle the return of waitForResult function
        if (mb_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("You have reached the destination");
            return true;             // True indicates that the goal was reached
        }
        else
        {
            ROS_INFO("The robot failed to reach the destination");
            return false;            // False indicates that this goal failed -> moving to next goal
        }
    }
    
    /*********************************************************************************************
     * @brief                   Publishes the final goals to rVIZ calculated to the goal_vec vector
     *                          which resides in the polyReciever method                     
     *********************************************************************************************/
        void goalpoints_marker(){

        float f = 0.0;                                              // Float definition

        visualization_msgs::Marker points;                          // Marker type definition

        points.header = _poly.header;                               // Header defintion
        points.ns = "goal_points";                                  // Namespace definition

            points.id = 2;                                          // Marker ID (unique)

        points.type = visualization_msgs::Marker::SPHERE_LIST;      // Type of marker

            // If goal vector is not empty
            if(!goal_vec.empty()){

            geometry_msgs::Point p;                                 // Point type definition

            // Iterate each position of goal vector
            // Convert this to a ROS float64 point and push to visualization_msgs point vector
            for (int i = 0; i < goal_vec.size(); i++)
            {
                p.x = goal_vec.at(i).x;
                p.y = goal_vec.at(i).y;

                points.points.push_back(p);
            }

            
            points.action = visualization_msgs::Marker::ADD;        // Setting the action to ADD for the publish message

            // DER MANGLER Marker::DELETE Statement ....
            
            // Defining the color of the goal points (green)
            points.color.g = 1.0f;
            points.color.a = 1.0;

            points.pose.orientation.w = 1.0;                        // Defining orientation of points

            points.scale.x = points.scale.y = 0.1;                  // Scaling points for a smaller size

            pointline_pub.publish(points);                          // Publish the points to rVIZ
            }
        return;
        }
    

    /*********************************************************************************************
     * @brief Function to build polygon from points recieved from rVIZ gui
     * @param point Point recieved from rviz
     * @param _poly "geometry_msgs::PolygonStamped"
     *********************************************************************************************/
    void polyReciever(const geometry_msgs::PointStampedConstPtr& point){
    // Calculation element for the last point of entry, to see if it is close to the first
    double average_distance = polygonPerimeter(_poly.polygon) / _poly.polygon.points.size();

    if(awaiting_center){
        //Flag is set so that this is the last point of the boundary polygon
            if(!pointInPolygon(point->point,_poly.polygon)){
            ROS_ERROR("Starting point is not inside polygon, restarting");
        }else{

            coor2d xy_temp;
            minmax mm_temp;
            s_goal goal_temp;
            s_goal goal_temp2;

                for (int i=0; i<_poly.polygon.points.size(); i++) 
            { 
                 // console out vector at current state (pre-sort)
                 std::cout << _poly.polygon.points.at(i).x << ", ";
                 std::cout << _poly.polygon.points.at(i).y << std::endl;
            }

            std::cout << "-------------------------------------------------" << std::endl;

            for (int i = 0; i < _poly.polygon.points.size(); i++)
            {
                xy_temp.x1 = _poly.polygon.points.at(i).x;
                xy_temp.y1 = _poly.polygon.points.at(i).y;

                vec.push_back(xy_temp);
            }

            std::cout << "vector elements: " << vec.size() << std::endl;
            
            std::cout << "  " << std::endl;

            std::cout << "Local vector points: " << std::endl;
            
            for (int i=0; i<vec.size(); i++) 
            { 
                 // console out vector at current state (pre-sort)
                 std::cout << vec.at(i).x1 << ", ";
                 std::cout << vec.at(i).y1 << std::endl;
            }
                xy_temp.x1 = vec.at(0).x1;
                xy_temp.y1 = vec.at(0).y1;
                xy_temp.x2 = vec.at(1).x1;
                xy_temp.y2 = vec.at(1).y1;
                float distance0_1 = xy_temp.distance_p1_p2();
                int steps0_1 = std::round(distance0_1 / 0.30f);

                xy_temp.x1 = vec.at(2).x1;
                xy_temp.y1 = vec.at(2).y1;
                xy_temp.x2 = vec.at(3).x1;
                xy_temp.y2 = vec.at(3).y1;
                float distance2_3 = xy_temp.distance_p1_p2();
                int steps2_3 = std::round(distance2_3 / 0.30f);

                std::cout << "Steps 1: " << steps0_1 << " " << std::endl;
                std::cout << "Steps 2: " << steps2_3 << " " << std::endl;

                std::cout << "Distance 1: " << distance0_1 << " Distance 2: " << distance2_3 << std::endl;                

                // The parametric equation for a line:
                // x = x1 + t * dx
                // y = y1 + t * dy 

                            // Point2        // Point1 
                double dx = (vec.at(1).x1) - (vec.at(0).x1);
                double dy = (vec.at(1).y1) - (vec.at(0).y1);
                double dist = sqrt(dx*dx + dy*dy);
                dx /= dist;
                dy /= dist;

                // Points from line0_1
                // first goal_vec.at(0) is the point which begins at point(0-3) 1 in the polygon
                for (int i = 0; i < steps0_1; i++)
                {
                goal_temp.x = vec.at(1).x1 + -robot_width*i * dx;
                goal_temp.y = vec.at(1).y1 + -robot_width*i * dy;
                
                    goal_vec_top.push_back(goal_temp);
                }

                      // Point2        // Point1 
                dx = (vec.at(2).x1) - (vec.at(3).x1);
                dy = (vec.at(2).y1) - (vec.at(3).y1);
                dist = sqrt(dx*dx + dy*dy);
                dx /= dist;
                dy /= dist;

                // Points from line2_3
                // first goal_vec.at(dependent on steps0_1) is the point which begins at point(0-3) 3 in the polygon
                for (int i = 0; i < steps2_3; i++)
                {
                goal_temp.x = vec.at(2).x1 + -robot_width*i * dx;
                goal_temp.y = vec.at(2).y1 + -robot_width*i * dy;
                
                    goal_vec_bot.push_back(goal_temp);                
                }

                std::cout << "TOP POINTS" << std::endl;
                for (int i=0; i<goal_vec_top.size(); i++) 
            { 
                 // console out vector at current state (pre-sort)
                 std::cout << goal_vec_top.at(i).x << ", ";
                 std::cout << goal_vec_top.at(i).y << std::endl;
            }

                std::cout << " " << std::endl;
                std::cout << "BOT POINTS" << std::endl;
                for (int i=0; i<goal_vec_bot.size(); i++) 
            { 
                // console out vector at current state (pre-sort)
                std::cout << goal_vec_bot.at(i).x << ", ";
                std::cout << goal_vec_bot.at(i).y << std::endl;
            }

            if (!goal_vec_top.empty() && !goal_vec_bot.empty()){

                auto v1 = goal_vec_top.begin ();
                auto v2 = goal_vec_bot.begin ();

                while (v1 != goal_vec_top.end () && v2 != goal_vec_bot.end ())
                {
                goal_vec.push_back(*v1);
                goal_vec.push_back(*v2);
                v1++;
                v2++;
                    if (v1 != goal_vec_top.end ())          // v1 is the longer one
                    {
                    break;
                    }else if (v2 != goal_vec_bot.end ())    // v2 is the longer one
                    {
                    break;
                    }
                goal_vec.push_back(*v2);
                goal_vec.push_back(*v1);
                v1++;
                v2++;
                }
                // if both the vectors have the same size we would be finished 
                if (v1 != goal_vec_top.end ()) // v1 is the longer one
                {
                    while (v1 != goal_vec_top.end ())
                    {
                    goal_vec.push_back (*v1);
                    ++v1;
                    }
                }
                if (v2 != goal_vec_bot.end ()) // v2 is the longer one
                {
                    while (v2 != goal_vec_bot.end ())
                    {
                    goal_vec.push_back (*v2);
                    ++v2;
                    }
                }
            }

            _poly.polygon.points.clear();

            ROS_INFO("Clearing polygon vector and initializing goal point marker");

            ros::Duration(0.2).sleep();

            goalpoints_marker();

            ros::spinOnce();

            //sleep added for the purpose of making a video (need to get to the camera)
            ros::Duration(8).sleep();

            donewaypoint = true;
            sortingdone = true;

            while (sortingdone = true)
            {
                if (donewaypoint == false )
                {
                    ros::spinOnce();
                }
                if (donewaypoint == true)
                    for (int i=0; i<=goal_vec.size(); i++) 
                    {
                    // console out vector at current state (pre-sort)
                    std::cout << "Sending movement goal to following coordinates: ";
                    
                    std::cout << goal_vec.at(i).x << ", ";

                    std::cout << goal_vec.at(i).y << std::endl;

                    std::cout << "Next goal is: ";

                    std::cout << goal_vec.at(i+1).x << ", ";

                    std::cout << goal_vec.at(i+1).y << std::endl;


                    goalReached = moveToGoal(goal_vec.at(i).x, goal_vec.at(i).y, goal_vec.at(i+1).x, goal_vec.at(i+1).y);
                    ROS_INFO("start moving");
                    ros::spinOnce();
                
                if (goalReached)
                {
                    ROS_INFO("Congratulations![%f, %f]", goal_vec.at(i).x, goal_vec.at(i).y);
                    DeminerClient::playSound(2,_sc);
                    ros::spinOnce();

                }else{
                    ROS_INFO("Movement and rotation goal failed!");
                    DeminerClient::playSound(3,_sc);
                }
                }
            }
        }
            ROS_INFO("Goal list is empty, beginning cleanup..");
            awaiting_center = false;
            sortingdone = false;

            goal_vec_top.clear();
            goal_vec_bot.clear();
            goal_vec.clear();
            _poly.polygon.points.clear();

            
    
        }else if(_poly.polygon.points.empty()){
            //first control point, so initialize header of boundary polygon
            _poly.header = point->header;
            _poly.polygon.points.push_back(costmap_2d::toPoint32(point->point));
            ROS_INFO("Header initialized and first point added to the database");

        }else if(_poly.header.frame_id != point->header.frame_id){
            ROS_ERROR("Frame mismatch, restarting polygon selection");
            _poly.polygon.points.clear();

        }else if(_poly.polygon.points.size() > 1 && pointsNearby(_poly.polygon.points.front(), point->point, average_distance*0.1)){
        
            //Check if the last point is nearby to the first point of the polygon
            
            if(_poly.polygon.points.size() < 3){
                ROS_ERROR("Not a valid polygon, restarting");
                _poly.polygon.points.clear();
            }else{
                awaiting_center = true;
                ROS_WARN("Please select a point inside the polygon, to start path planning");
                DeminerClient::playSound(1,_sc);
            }

        }else{

            //otherwise, it must be a regular point inside boundary polygon
            //push_back point to a vector (can be used later on..)
            _poly.polygon.points.push_back(costmap_2d::toPoint32(point->point));
            _poly.header.stamp = ros::Time::now();
            ROS_INFO("Point added to the database");
        }
        //send_goal();
    }
        // // // Do not delete
        void boundingBox(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
        {
            qX = msg->pose.pose.position.x; // Robot X position
            qY = msg->pose.pose.position.y; // Robot Y position
            qZ = msg->pose.pose.position.z;  // Robot Z posistion
            qYaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
            
            std::cout << "X: " << qX << " Y: " << qY << " Z: " << qZ << " Yaw: " << qYaw << std::endl;

            return;
        }


public:

    /**
     * @brief Constructor for the deminer client
     * Initializing the client handlers, publishers and subscribers
     */
    DeminerClient() :
        _n(),
        _sc(),
        mb_client("move_base", true),
        awaiting_center(false),
        goalReached(false),
        donewaypoint(false),
        sortingdone(false)
    {
        coor2d temp;
        minmax mm_temp;
        s_goal goal_temp;

        _poly.header.frame_id = "map";

        pointclick_sub = _n.subscribe("/clicked_point",10,&DeminerClient::polyReciever, this);

        amcl_pose_sub = _n.subscribe("amcl_pose", 100, &DeminerClient::boundingBox, this);

        goal_m_pub = _n.advertise<visualization_msgs::Marker>("goal_points", 10);

        pointline_pub = _n.advertise<visualization_msgs::Marker>("demining_polygon_marker", 10);

        pointline_timer = _n.createWallTimer(ros::WallDuration(0.1), boost::bind(&DeminerClient::pointLines, this));

        //boundingbox_pub = _n.advertise<visualization_msgs::Marker>("bounding_box", 10);

        ROS_INFO("Please stand by, while the client initializes..");
        //Sleep needed for the initialization of SoundClient
        ros::Duration(2).sleep();
        ROS_INFO("Client initialized!");

        DeminerClient::playSound(0,_sc);

        ROS_INFO("Please use the 'Point' tool in Rviz to select an demining area.");
    }  
};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deminer_client");

    deminer::DeminerClient clientclientclientclientclient;

    ros::spin();
    return 0;
}