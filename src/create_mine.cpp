#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h> // For testing
#include <geometry_msgs/PolygonStamped.h> // For testing
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Int8.h"

#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>

#include <stdint.h>


namespace create_mine {

    class Flag
    {
    private:
	ros::NodeHandle n_;
	ros::Publisher pub_point;
	ros::Publisher pub_settings;
    ros::Publisher pub_markers;
    ros::Subscriber sub_mine;
    ros::Subscriber amcl_pose_sub;
    ros::Subscriber sensorAndSafetyCircuit; 

    geometry_msgs::PolygonStamped _poly;
    geometry_msgs::Point point_;
    geometry_msgs::Point settings_;
	
	bool operation_;
    bool remove_all_;
    bool size_property_;
    float mine_size_;
    float qX;
    float qY;


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

    void getPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
        {
            qX = msg->pose.pose.position.x; // Robot X position
            qY = msg->pose.pose.position.y; // Robot Y position
            
            std::cout << "X: " << qX << " Y: " << qY << std::endl;

            return;
        }

    void publishMine(std_msgs::Int8 data){

            int operation;

            operation = data.data;

            if (operation == 0){
            ros::spinOnce;
            }else
            {
            ros::Rate publish_rate(20);

            std_msgs::Int8 operation;

            operation_=false;

            settings_.x = 1;
            settings_.y = false; //clear
            settings_.z = 1; //color

            s_mine m; 

            m.x = qX;
            m.y = qY; 

            // m.yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw (Not used)
            
            std::cout << "PublishMine: " << "X: " << m.x << " Y: " << m.y << " Z: " << m.z << std::endl;

            minevector.push_back(m);

            point_.x = m.x;
            point_.y = m.y;
            point_.z = operation_;
            //pub_settings.publish(settings_);
            //pub_point.publish(point_);

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
            
    public:
    Flag() :
    n_(),
    operation_(true)
    {

    pub_point = n_.advertise<geometry_msgs::Point>("/detected_mine",10);
    pub_settings = n_.advertise<geometry_msgs::Point>("/restrict_settings",10);
    pub_markers = n_.advertise<visualization_msgs::Marker>("mine_markers", 10);
    // sub_mine = n_.subscribe("/clicked_point",1, &Flag::publishMine, this); // For testing purposes.
    amcl_pose_sub = n_.subscribe("amcl_pose", 100, &Flag::getPose, this);
    sensorAndSafetyCircuit = n_.subscribe("sensorAndSafetyTopic", 100, &Flag::publishMine, this);
    }
    
    ~Flag() {}
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_mine");

    create_mine::Flag flagger;

    std::cout << "create mine interface" << std::endl;

    ros::spin();
    
    return 0;
}