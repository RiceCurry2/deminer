// This was not finished due to a constrain of time, but the purpose of this node was to divide the software into a client node and a server node

#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h> // For testing
#include <geometry_msgs/PolygonStamped.h> // For testing

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>

namespace deminer {

    class DeminerServer
    {
    private:
	ros::NodeHandle n_;
	ros::Publisher pub_point;
	ros::Publisher pub_settings;
    ros::Publisher pub_markers;
    ros::Subscriber sub_mine;

    geometry_msgs::PolygonStamped _poly;
    geometry_msgs::Point point_;
    geometry_msgs::Point settings_;

    // Actionlib "move_base" definitons 
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> mb_server;
    move_base_msgs::MoveBaseGoal goal;
	
	bool operation_;

    struct s_mine
    {
        float x;
        float y;
        float z;
    };
    

    std::vector<s_mine> minevector;

    void mine_markers(){

    float f = 0.0;                                                  // Float definition

    visualization_msgs::Marker points;                              // Marker type definition

    points.header.frame_id = "map";
    points.header.stamp = ros::Time();
    points.ns = "mine_detected";                                    // Namespace definition

            points.id = 3;                                          // Marker ID (unique)

        points.type = visualization_msgs::Marker::SPHERE_LIST;      // Type of marker

            // If goal vector is not empty
            if(!minevector.empty()){

            geometry_msgs::Point p;                                 // Point type definition

            // Iterate each position of goal vector
            // Convert this to a ROS float64 point and push to visualization_msgs point vector
            for (int i = 0; i < minevector.size(); i++)
            {
                p.x = minevector.at(i).x;
                p.y = minevector.at(i).y;

                points.points.push_back(p);
            }

            
            points.action = visualization_msgs::Marker::ADD;        // Setting the action to ADD for the publish message

            // DER MANGLER Marker::DELETE Statement ....
            
            // Defining the color of the goal points (green)
            points.color.r = 1.0;
            points.color.a = 1.0;

            points.pose.orientation.w = 1.0;                        // Defining orientation of points

            points.scale.x = points.scale.y = 0.2;                  // Scaling points for a smaller size

            pub_markers.publish(points);                            // Publish the points to rVIZ
            }
        return;
        }

    void publishMine(const geometry_msgs::PointStampedConstPtr& point){

            ros::Rate publish_rate(20);

            if (operation_){

            operation_=false;

            settings_.x = 1;
            settings_.y = false; //clear
            settings_.z = 1; //color

            s_mine m; 

            m.x = -0.78333;
            m.y = 2.16096;
            m.z = 1;

            minevector.push_back(m);

            point_.x = m.x;
            point_.y = m.y;
            point_.z = operation_;
            pub_settings.publish(settings_);
            pub_point.publish(point_);

            publish_rate.sleep();

            ros::spinOnce;

            mine_markers();

            ROS_INFO("Mine location published and saved to vector");


            operation_=true;

            //}else{
            //ROS_INFO("Awaiting mine location");
            //ros::spinOnce;

        return;
        }     
    }

    void execute(const move_base_msgs::MoveBaseGoalConstPtr& goal)
    {

    std::cout << "goal was successful" << std::endl;

    mb_server.setSucceeded();
    }

            
    public:
    DeminerServer() :
    n_(),
    operation_(true),
    mb_server(n_, "DeminerServer", boost::bind(&DeminerServer::execute, this, _1), false)
    {
    pub_point = n_.advertise<geometry_msgs::Point>("/detected_mine",10);
    pub_settings = n_.advertise<geometry_msgs::Point>("/restrict_settings",10);
    pub_markers = n_.advertise<visualization_msgs::Marker>("mine_markers", 10);
    sub_mine = n_.subscribe("/clicked_point",1, &DeminerServer::publishMine, this);


    mb_server.start();
    }
    
    ~DeminerServer() {}
    };
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "deminer_server");

    deminer::DeminerServer server;

    std::cout << "create mine interface" << std::endl;

    ros::spin();
    
    return 0;
}


