#include<ros/ros.h>
#include<sensor_msgs/PointCloud.h>
#include<point.h>
#include<segment.h>
#include<circle.h>
using namespace sensor_fusion;

#define PI 3.141592
class lidar_obstacle{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_pointcloud;
        sensor_msgs::PointCloud bound_point;
        std::list<Point> point_set;
        std::list<Segment> segment_set;
        std::list<Circle> circle_set;
        std::list<Point>::iterator p_iter;
        double max_distance = 10;
        double min_distance = 0.5;
        double point_distance = 0.2;
        int angle_threshold = 30;
        
    public:
        lidar_obstacle();
        ~lidar_obstacle();
        void pointCallback(sensor_msgs::PointCloud p);   // check 
        void detect_segment();
        void detect_circle();

};