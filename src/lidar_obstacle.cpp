#include<lidar_obstacle.h>

lidar_obstacle::lidar_obstacle(){
    sub_pointcloud = n.subscribe("/scan_pointcloud",1000,&lidar_obstacle::pointCallback, this);
}
lidar_obstacle::~lidar_obstacle(){}

void lidar_obstacle::pointCallback(sensor_msgs::PointCloud p){
    for(auto& c : p.points){
        if(c.x < max_distance && c.x > min_distance && c.y < max_distance && c.y > min_distance){
            Point tmp;
            tmp.x = c.x; tmp.y = c.y;
            point_set.push_back(tmp);
        }
    }
}

void lidar_obstacle::detect_segment(){
    std::list<Point>::iterator f_p = point_set.begin();
    std::list<Point>::iterator l_p = point_set.begin();
    std::list<Point>::iterator c_p = point_set.begin();
    double _min = 11; 

    for(p_iter = point_set.begin(); p_iter != point_set.end(); p_iter++){
        double distance = (*p_iter-*l_p).length();
        if(distance <= point_distance){
            l_p = p_iter;
            if((*l_p).length() < _min)
                c_p = l_p;
        }
        else{
            double a_cos =(pow((*f_p).length(),2) + pow((*l_p).length(),2) - pow((*l_p - *f_p).length(),2))/(2.0*((*f_p).length())*((*l_p).length()));
            double angle = acos(a_cos) * 180 / PI;
            if(angle >= angle_threshold){
                Segment s = Segment(*f_p,*l_p,*c_p);
                segment_set.push_back(s);
            }
        }

    }
}

int main(){

}

