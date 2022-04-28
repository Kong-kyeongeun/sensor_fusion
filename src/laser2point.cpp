#include<laser2point.h>
#include<tf_broadcast.h>

laser2point::laser2point(){
    pub_point = n.advertise<sensor_msgs::PointCloud>("/scan_pointcloud", 1000);
    sub_lidar = n.subscribe("/front_scan", 1000, &laser2point::scanCallback, this);
    n.getParam("/tf_broadcast/from_frame_id", from_frame_id);
    n.getParam("/tf_broadcast/to_frame_id", to_frame_id);
    sub_laser= false;
}

laser2point::~laser2point(){}
void laser2point::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_lsc_ = *scan;
    sub_laser= true;
    tf_ls_.waitForTransform(sensor_lsc_.header.frame_id, from_frame_id,
        sensor_lsc_.header.stamp + ros::Duration().fromSec(sensor_lsc_.ranges.size() * sensor_lsc_.time_increment), ros::Duration(0.05));
  
    projector_.transformLaserScanToPointCloud(from_frame_id , sensor_lsc_, sensor_pcl_, tf_ls_);
    sensor_pcl_.header.frame_id = to_frame_id;
}


void laser2point::pub_pointcloud(){
    if(sub_laser){
        pub_point.publish(sensor_pcl_);
    }
}

// void laser2point::tf_laser2point(){

//      if(sub_laser&& laser2point::get_transform() ){
// 	 		tf::transformStampedTFToMsg(tf_between_frames, tf_between_frames_geo); // convert it to geometry_msg type
// 	 		sensor_msgs::convertPointCloudToPointCloud2(sensor_pcl_, sensor_pcl_2); // convert from pointcloud to pointcloud2
// 	 		tf2::doTransform(sensor_pcl_2, tf_sensor_pcl_2, tf_between_frames_geo); // do transformation
//              sensor_msgs::convertPointCloud2ToPointCloud(tf_sensor_pcl_2, sensor_pcl_output);
//      }

// }

// bool laser2point::get_transform(){
//     if (!tf_ls_.waitForTransform(from_frame_id, to_frame_id, ros::Time(0), ros::Duration(0.5))) {
// 	 		return false;
// 	 } 
//     else {
// 		try {
// 			tf_ls_.lookupTransform(from_frame_id, to_frame_id, ros::Time(0), tf_between_frames);
// 		}
// 		catch (const tf::TransformException &e) {
// 			return false;
// 		}
// 	}
//     return true;
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_laser2point");
    laser2point l2p;
    ros::Rate r(1000);
    while(ros::ok()){
//        l2p.tf_laser2point();
        l2p.pub_pointcloud();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
