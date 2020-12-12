#include "ros/ros.h"
#include <geometry_msgs/Point.h>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>


namespace create_mine {

    class Flag
    {
    private:
	ros::NodeHandle n_;
	ros::Publisher pub_point;
	ros::Publisher pub_settings;
    ros::Subscriber sub_mine;
    geometry_msgs::Point point_;
    geometry_msgs::Point settings_;
	
	bool operation_;
    bool remove_all_;
    bool size_property_;
    float mine_size_;

    struct s_mine
    {
        float x;
        float y;
        float z;
    };
    

    std::vector<s_mine> minevector;

    void publishMine(geometry_msgs::Point p){

            if (operation_=false){
            operation_=true;

            s_mine m; 

            m.x = p.x; 
            m.y = p.y;
            m.z = p.z; 

            minevector.push_back(m);

            point_.x = p.x; 
            point_.y = p.y;
            point_.z = p.z;

            pub_point.publish(point_);
            pub_settings.publish(settings_);

            ROS_INFO("Mine location published and saved to vector");

            operation_=false;


            return;
            }else{
            ROS_INFO("Awaiting mine location");
            ros::spinOnce;
            }
            
    }

void updateTopic(int a)
{
    // Switch statement to handle the sound files
    switch(a)
    {
    case 0:
        settings_.x = mine_size_;
        settings_.y = true;                   //clear
        settings_.z = 1;                      //color
        return;
    case 1:
        settings_.x = mine_size_;
        settings_.y = false; //clear
        settings_.z = 1;                      //color
        return;
    case 2:
        mine_size_ = 2;
        return;
    case 3:
        mine_size_ = 4;
        return;
    default:
        ROS_ERROR("Error: The updateTopic recieved an undefined number");
    }
    return;
}

            
    public:
    Flag() :
    n_(),
    operation_(false)
    {
    

    pub_point = n_.advertise<geometry_msgs::Point>("/mine_location",1);

    pub_settings = n_.advertise<geometry_msgs::Point>("/mine_settings",1);

    sub_mine = n_.subscribe<geometry_msgs::Point>("/mine_publish",1, &Flag::publishMine, this);
    }
    
    ~Flag() {}
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_mine");

    create_mine::Flag flagger;

    ros::spin();
    
    return 0;
}