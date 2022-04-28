
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"

class tf_broadcast{
    private:
        ros::NodeHandle n;
        tf::TransformBroadcaster broadcast_tf;
        // ros::Subscriber sub_car;
        double sensor_offset_x, sensor_offset_y, sensor_offset_z, sensor_offset_yaw;
        std::string from_frame_id;
        std::string to_frame_id; 
        tf::Quaternion car2sensor_q_;
        tf::Transform car2sensor_transform_;

    public:
        tf_broadcast();
        ~tf_broadcast();

        void broadcast_tf_all();

};