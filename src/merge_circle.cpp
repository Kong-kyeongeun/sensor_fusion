#include"merge_circle.h"

#define PI 3.141592

merge_circle::merge_circle(){
    markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/viz_lidar_circle",1000);
    camera_markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/viz_camera_circle",1000);
    merge_markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/viz_merge_circle",1000);
    sub_lidar_circle = n.subscribe("/obstacles", 1000, &merge_circle::lidarCallback, this);
    
}
merge_circle::~merge_circle(){

}

visualization_msgs::Marker merge_circle::markergenerator(geometry_msgs::PoseStamped point,double scale,double scale_z, int _id, double color){
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

        marker.scale.x = scale*2; //임의의 비행금지구역 반경 (x,y 같게)
        marker.scale.y = scale*2;
        marker.scale.z = scale_z; //비행금지구역 높이는 무제한

        marker.color.r = 1.0f;
        marker.color.g = color;
        marker.color.b = 0.0f;
        marker.color.a = 0.5; //마커 색상 및 투명도

        marker.lifetime = ros::Duration(0.1); //마커 지속시간
        marker.header.frame_id = "/map";
        return marker;
    }
    
void merge_circle::viz_lidar_circle(){
    D_WPArray.markers.clear();
    num = 0;
    for (auto& r : lidar_circle.circles){
        wp.pose.position.x = r.center.x;
        wp.pose.position.y = r.center.y;
        D_WPArray.markers.push_back(markergenerator(wp,r.radius,0.1,num,1.0));
        num++;
    }
    markerArrayPub.publish(D_WPArray);
}

void merge_circle::viz_merge_circle(){
    D_WPArray.markers.clear();
    for (auto& r : _merge_circle.circles){
        wp.pose.position.x = r.center.x;
        wp.pose.position.y = r.center.y;
        D_WPArray.markers.push_back(markergenerator(wp,r.radius,0.1,num,0.5));
        num++;
    }
    merge_markerArrayPub.publish(D_WPArray);
}

void merge_circle::viz_camera_circle(){
    D_WPArray.markers.clear();
    wp.pose.position.x = 0.05;
    wp.pose.position.y = 0.3;
    D_WPArray.markers.push_back(markergenerator(wp,0.2,0.1,num,0.0));
    camera_markerArrayPub.publish(D_WPArray);
    num++;
}

void merge_circle::lidarCallback(sensor_fusion::Obstacles _lidar_circle ){
    lidar_circle = _lidar_circle;
}

void merge_circle::CameraCallback(){

}

void merge_circle::merge(){
    _merge_circle.circles.clear();
    int j =0;
    for (auto& r : lidar_circle.circles){
        double distance = pow(pow(r.center.x-0.05,2)+pow(r.center.y-0.3,2),0.5);
        if(distance-0.2 < r.radius){
            _merge_circle.circles.push_back(r);
            if(distance+0.2 >= r.radius)
                _merge_circle.circles[j].radius = distance + 0.2;
            j++;
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "merge_sensor");
    merge_circle mer;
    ros::Rate r(1000);
    while(ros::ok()){
        mer.merge();
        mer.viz_lidar_circle();
        mer.viz_camera_circle();
        mer.viz_merge_circle();
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

