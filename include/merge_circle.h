#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_fusion/Obstacles.h"
#include "sensor_fusion/CircleObstacle.h"
#include "sensor_fusion/SegmentObstacle.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class merge_circle{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_lidar_circle;
        ros::Subscriber sub_camera_circle;
        ros::Publisher markerArrayPub;

    public:
        merge_circle();
        ~merge_circle();

        void lidarCallback(sensor_fusion::Obstacles  lidar_circle );
        visualization_msgs::Marker markergenerator(geometry_msgs::PoseStamped point,double scale,double scale_z, int _id);

};

