#include<tf_broadcast.h>

tf_broadcast::tf_broadcast(){
    sensor_offset_x = 3;
    sensor_offset_y = 0;
    sensor_offset_z = 0;
    sensor_offset_yaw = 0;
    car_frame_id = "/laser_frame";
    car2sensor_q_.setRPY(0, 0, sensor_offset_yaw);
    car2sensor_q_.normalize();
    car2sensor_transform_.setOrigin(tf::Vector3(sensor_offset_x,sensor_offset_y, sensor_offset_z));
    car2sensor_transform_.setRotation(car2sensor_q_);
    
}
tf_broadcast::~tf_broadcast(){}

void tf_broadcast::broadcast_tf_all(){
      broadcast_tf.sendTransform(tf::StampedTransform(
      car2sensor_transform_, ros::Time::now(), "/laser_frame", "/map"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_tf");
    tf_broadcast br;
    ros::Rate r(1000);
    while(ros::ok()){
        br.broadcast_tf_all();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
    
