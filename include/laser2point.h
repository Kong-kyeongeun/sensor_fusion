#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
// #include "geometry_msgs/TransformStamped.h"
// #include "nodelet/nodelet.h"
// #include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class laser2point{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_lidar;
        ros::Publisher pub_point;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tf_ls_;
        tf::StampedTransform tf_between_frames;

        sensor_msgs::PointCloud sensor_pcl_;
        sensor_msgs::PointCloud sensor_pcl_output;
        sensor_msgs::PointCloud2 sensor_pcl_2;
        sensor_msgs::PointCloud2 tf_sensor_pcl_2;
        sensor_msgs::LaserScan sensor_lsc_;
        geometry_msgs::TransformStamped tf_between_frames_geo;
        


        std::string to_frame_id; 
        std::string from_frame_id;

        bool sub_laser;

        
    public:
        laser2point();
        ~laser2point();

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void tf_laser2point();
        void pub_pointcloud();
        bool get_transform();
};