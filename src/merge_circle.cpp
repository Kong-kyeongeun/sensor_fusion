#include"merge_circle.h"

#define PI 3.141592

merge_circle::merge_circle(){
    markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/viz_lidar_circle",1000);
    sub_lidar_circle = n.subscribe("/obstacles", 1000, &merge_circle::lidarCallback, this);
}
merge_circle::~merge_circle(){

}

visualization_msgs::Marker merge_circle::markergenerator(geometry_msgs::PoseStamped point,double scale,double scale_z, int _id){
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.ns = "basic_shapes";
        marker.id = _id;
        marker.pose.position.x = point.pose.position.x;
        marker.pose.position.y = point.pose.position.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = point.pose.position.z*2; //임의의 비행금지구역 반경 (x,y 같게)
        marker.scale.y = point.pose.position.z*2;
        marker.scale.z = scale_z; //비행금지구역 높이는 무제한

        marker.color.r = 1.0f;
        marker.color.g = 1;
        marker.color.b = 0.0f;
        marker.color.a = 0.5; //마커 색상 및 투명도

        marker.lifetime = ros::Duration(); //마커 지속시간
        marker.header.frame_id = "/map";
        return marker;
    }
    
void merge_circle::viz_lidar_circle(){
    D_WPArray.markers.clear();
    int num = 0;
    for (auto& r : lidar_circle.circles){
        wp.pose.position.x = r.center.x;
        wp.pose.position.y = r.center.y;
        wp.pose.position.z = r.radius;
        D_WPArray.markers.push_back(markergenerator(wp,0.1,0.1,num));
        num++;
    }
    markerArrayPub.publish(D_WPArray);

};

void merge_circle::lidarCallback(sensor_fusion::Obstacles  _lidar_circle ){
    lidar_circle = _lidar_circle;
}

void merge_circle::cameraCallback(){

}

int main(int argc, char** argv){
    ros::init(argc, argv, "merge_sensor");
    merge_circle mer;
    ros::Rate r(1000);
    while(ros::ok()){
        viz_lidar_circle();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

// geometry_msgs/Point center      # Central point [m]
// geometry_msgs/Vector3 velocity  # Linear velocity [m/s]
// float64 radius                  # Radius with added margin [m]
// float64 true_radius             # True measured radius [m]
// float64 angle			# Angle of Center [degree]
// float64 c_distance		# Distance from Center [m]
// float64 min_distance		# Min_Distance [m]

// Header header

// obstacle_detector/SegmentObstacle[] segments
// obstacle_detector/CircleObstacle[] circles

