#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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
 * @brief           Private definitions
 * @param _point    Namespace: geometry_msgs::PointStamped
 * @param _poly     Namespace: geometry_msgs::PolygonStamped
 *********************************************************************************************/
private:
    // Defining handlers, publishers and subcribers
    ros::NodeHandle _n;
    sound_play::SoundClient _sc;

    geometry_msgs::PolygonStamped _poly;
    geometry_msgs::PointStamped _point; //maybe not used
    geometry_msgs::Point32 _as; //maybe not used

    ros::Publisher pointline_pub;
    ros::Publisher goal_m_pub;
    ros::Publisher boundingbox_pub; //maybe not used
    ros::Subscriber pointclick_sub;
    ros::Subscriber amcl_pose_sub;
    ros::WallTimer pointline_timer;
    ros::WallTimer goal_m_timer;


    std::string path_to_sounds = "../catkin_ws/src/deminer/sounds/";

    bool awaiting_center;
    float robot_width = 0.30f;

    struct Vector2 //maybe not used
    {
        float m_x;
        float m_y;

        Vector2(float x, float y)
        : m_x(x), m_y(y) {}

        };

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

    struct goal {
        float x; 
        float y;
    };


    std::vector<coor2d> vec;

    std::vector<goal> goal_vec;

    //Maybe not used: std::vector<minmax> v_minmax;

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
 * This is a function for the purpose of playing sound 
 * The '&' represents that the sound client publisher are passed by reference from the main.
 * This is done for the purpose of using the 'sc.' nameclasses outside the main loop.
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
        _sc.playWave(path_to_sounds + "boundary_selection.wav");
        return;
    default:
        ROS_ERROR("Error: The playsound function recieved an undefined number");
    }

    return;
}

   /**
     * @brief Publish markers for visualization of points for boundary polygon.
    */
    void pointLines(){

    float f = 0.0;

    visualization_msgs::Marker points, line_strip;

    points.header = line_strip.header = _poly.header;
    points.ns = line_strip.ns = "boundary_points";

        points.id = 0;
        line_strip.id = 1;

    points.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    if(!_poly.polygon.points.empty()){

        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        points.scale.x = points.scale.y = 0.1;
        line_strip.scale.x = 0.05;

        BOOST_FOREACH(geometry_msgs::Point32 point, _poly.polygon.points){
        line_strip.points.push_back(costmap_2d::toPoint(point));
        points.points.push_back(costmap_2d::toPoint(point));
        }

        if(awaiting_center){
            line_strip.points.push_back(costmap_2d::toPoint(_poly.polygon.points.front()));
            // RED Color (Points and lines)
            //OLD//points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
            points.color.r = 1.0;
            points.color.a = 1.0;
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;

        }else{
            // GREEN Color (Points and lines)
            //OLD//points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0; 
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
            // Creating new item "temp" in struct
            coor2d xy_temp;
            minmax mm_temp;
            goal goal_temp;

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
                // std::cout << vec.at(0).x1 << vec.at(0).y1 << std::endl;
                                
                // std::cout << vec.at(1).x1 << vec.at(1).y1 << std::endl;

                // std::cout << vec.at(2).x1 << vec.at(2).y1 << std::endl;
                                
                // std::cout << vec.at(3).x1 << vec.at(3).y1 << std::endl;

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
                // // // float px = fabsf(mm_temp.min_x);
                // // // float py = fabsf(mm_temp.min_y);
                // // // std::cout << "Count to plus: " << px << ", " << py << std::endl;

                // // // float sum = 0;

                // // // sum = px + mm_temp.max_x;
                // // // std::cout << "low + high: " << px << " + " << mm_temp.max_x << " = " << sum <<  std::endl;
                // // // int cx = std::round(sum / 0.30f);
                // // // std::cout << "x count: " << cx << std::endl;

                // // // sum = py + mm_temp.max_y;
                // // // std::cout << "low + high: " << py << " + " << mm_temp.max_y << " = " << sum <<  std::endl;
                // // // int cy = std::round(sum / 0.30f);
                // // // std::cout << "y count: " << cy << std::endl;

                // The parametric equation for a line:
                // x = x1 + t * dx
                // y = y1 + t * dy 
                // ------------------------------------------------------------------------------------------------------
                            // Point2        // Point1 
                double dx = (vec.at(1).x1) - (vec.at(0).x1);
                double dy = (vec.at(1).y1) - (vec.at(0).y1);
                double dist = sqrt(dx*dx + dy*dy);
                dx /= dist;
                dy /= dist;

                for (int i = 0; i < steps0_1; i++)
                {
                goal_temp.x = vec.at(1).x1 + -robot_width*i * dx;
                goal_temp.y = vec.at(1).y1 + -robot_width*i * dy;
                
                    goal_vec.push_back(goal_temp);
                }

                            // Point2        // Point1 
                dx = (vec.at(3).x1) - (vec.at(2).x1);
                dy = (vec.at(3).y1) - (vec.at(2).y1);
                dist = sqrt(dx*dx + dy*dy);
                dx /= dist;
                dy /= dist;

                for (int i = 0; i < steps2_3; i++)
                {
                goal_temp.x = vec.at(3).x1 + -robot_width*i * dx;
                goal_temp.y = vec.at(3).y1 + -robot_width*i * dy;
                
                    goal_vec.push_back(goal_temp);                
                }

                for (int i=0; i<goal_vec.size(); i++) 
            { 
                 // console out vector at current state (pre-sort)
                 std::cout << goal_vec.at(i).x << ", ";
                 std::cout << goal_vec.at(i).y << std::endl;

            }
                

                //std::cout << vec.at(1).x1 << " - " << vec.at(0).x1 << " = " << dx << std::endl;
                //std::cout << vec.at(1).y1 << " - " << vec.at(0).y1 << " = " << dy << std::endl;
                //std::cout << " " << std::endl;
                //std::cout << "Performed calculations: " << std::endl;

                //std::cout << xy_temp.x1 << ", " << xy_temp.y1 << std::endl;


            // Vector sorts for min/max
            // --------------------------------------------------------------------------
            // // // // Call sort funcion (sorting for lowest x pt)
            // // // // Thus..   vec.at(3) = highest X
            // // // //          vec.at(0) = lowest X
            // // // std::sort(vec.begin(), vec.end(), compare_xy());
            // // // std::cout << std::endl;

            // // //     // Load max/min variables into vector via temp struct
            // // //     mm_temp.min_x = vec.at(0).x;
            // // //     mm_temp.max_x = vec.at(3).x;

            // // //     //Maybe not used: v_minmax.push_back(mm_temp);

            // // // std::cout << " " << std::endl;
            // // // std::cout << "v_ " << std::endl;

            // // // std::cout << " " << std::endl;
            // // // std::cout << "Sorted vector points XY: " << std::endl;
            
            // // // for (int i=0; i<vec.size(); i++) 
            // // // { 
            // // //      // console out vector at current state (pro-sort)
            // // //      // 
            // // //      std::cout << vec.at(i).x << ",  ";
            // // //      std::cout << vec.at(i).y << std::endl;
            // // // }

            // // // // Call sort funcion (sorting for lowest y pt)
            // // // // Thus..   vec.at(3) = highest Y
            // // // //          vec.at(0) = lowest Y
            // // // std::sort(vec.begin(), vec.end(), compare_yx());
            // // // std::cout << std::endl;

            // // //     // Load max/min variables into vector via temp struct
            // // //     mm_temp.min_y = vec.at(0).y;
            // // //     mm_temp.max_y = vec.at(3).y;

            // // //     //Maybe not used: v_minmax.push_back(mm_temp);


            // // // std::cout << "Sorted vector points YX: " << std::endl;
            
            // // // for (int i=0; i<vec.size(); i++) 
            // // // { 
            // // //      // console out vector at current state (pro-sort)
            // // //      // 
            // // //      std::cout << vec.at(i).x << ",  ";
            // // //      std::cout << vec.at(i).y << std::endl;
            // // // }
         
            // // //     std::cout << " " << std::endl;
            // // //      // Print min singular points
            // // //     std::cout << "min x: " << mm_temp.min_x << ",  ";
            // // //     std::cout << "min y: " << mm_temp.min_y << std::endl;

            // // //      // Print max singular points
            // // //     std::cout << "max x: " << mm_temp.max_x << ",  ";
            // // //     std::cout << "max y: " << mm_temp.max_y << std::endl;

            // // //     //vec.clear();
            // // //     //ROS_INFO("Sorting vector has been cleared");

            // // //     //min x: -0.490582,  min y: -1.0317
            // // //     //max x: 2.18035,  max y: 1.527

                // MIN/MAX (Maybe not used)
                //-------------------------------------------------------------------------------------------------------
                // // float px = fabsf(mm_temp.min_x);
                // // float py = fabsf(mm_temp.min_y);
                // // std::cout << "Count to plus: " << px << ", " << py << std::endl;

                // // float sum = 0;

                // // sum = px + mm_temp.max_x;
                // // std::cout << "low + high: " << px << " + " << mm_temp.max_x << " = " << sum <<  std::endl;
                // // int cx = std::round(sum / 0.30f);
                // // std::cout << "x count: " << cx << std::endl;

                // // sum = py + mm_temp.max_y;
                // // std::cout << "low + high: " << py << " + " << mm_temp.max_y << " = " << sum <<  std::endl;
                // // int cy = std::round(sum / 0.30f);
                // // std::cout << "y count: " << cy << std::endl;
                //-------------------------------------------------------------------------------------------------------

                // if (cy < cx)
                // {
                //     vec.reserve(cx*2);
                //     std::cout << "Reserved " << cx*2 << " in vector" << std::endl;

                // }else if (cy >= cx){

                //     vec.reserve(cy*2);
                //     std::cout << "Reserved " << cy*2 << " in vector" << std::endl;

                // }

                // for (int i=0; i<cx; i++)
                // {
                // xy_temp.x += mm_temp.min_x + robot_width;
                // std::cout << i << " " << mm_temp.min_x << " + " << robot_width << " = " << xy_temp.x << std::endl;
                // vec.push_back(xy_temp);
                // }

                // for (int i=0; i<cy; i++)
                // {
                // xy_temp.y += mm_temp.min_y + robot_width;
                // std::cout << i << " " << mm_temp.min_y << " + " << robot_width << " = " << xy_temp.y << std::endl;
                // vec.push_back(xy_temp);
                // }

                // for (int i=0; i<vec.size(); i++) 
                // { 
                //     std::cout << vec.at(i).x << ",  ";
                //     std::cout << vec.at(i).y << std::endl;
                // }

        ROS_WARN("Waiting for the move_base action server to come up");
        mm_temp.reset();

        }
            awaiting_center = false;
            _poly.polygon.points.clear();
            vec.clear();
            
            //Maybe not used: v_minmax.clear();
            
    
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
    }

        void boundingBox(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
        {
            double X = msg->pose.pose.position.x; // Robot X postition
            double Y = msg->pose.pose.position.y; // Robot Y postition
            double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

            std::cout << "X: " << X << " Y: " << Y << " Z: " << Yaw << std::endl;

            return;
        }

        void goalpoints_marker(){

            float f = 0.0;

            visualization_msgs::Marker points;

            points.header = _poly.header;
            points.ns = "goal_points";

                points.id = 2;

            points.type = visualization_msgs::Marker::SPHERE_LIST;

            if(!goal_vec.empty()){

            geometry_msgs::Point p;

            for (int i = 0; i < goal_vec.size(); i++)
            {
                p.x = goal_vec.at(i).x;
                p.y = goal_vec.at(i).y;
        
                points.points.push_back(p);
            }

            points.action = visualization_msgs::Marker::ADD;

            // DER MANGLER Marker::DELETE Statement ....

            points.color.g = 1.0f;
            points.color.a = 1.0;

            points.pose.orientation.w = 1.0;

            points.scale.x = points.scale.y = 0.1;

            pointline_pub.publish(points);
            }
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
        awaiting_center(false)
    {
        coor2d temp;
        minmax mm_temp;
        goal goal_temp;

        _poly.header.frame_id = "map";

        pointclick_sub = _n.subscribe("/clicked_point",10,&DeminerClient::polyReciever, this);

        amcl_pose_sub = _n.subscribe("amcl_pose", 100, &DeminerClient::boundingBox, this);

        goal_m_pub = _n.advertise<visualization_msgs::Marker>("goal_points", 10);

        goal_m_timer = _n.createWallTimer(ros::WallDuration(0.1), boost::bind(&DeminerClient::goalpoints_marker, this));

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

    deminer::DeminerClient client;

    ros::spin();
    return 0;
}