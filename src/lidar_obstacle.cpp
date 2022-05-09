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
            if((*l_p).length() < _min){
                c_p = l_p;
                _min = (*l_p).length();
            }
        }
        else{
            double a_cos =(pow((*f_p).length(),2) + pow((*l_p).length(),2) - pow((*l_p - *f_p).length(),2))/(2.0*((*f_p).length())*((*l_p).length()));
            double angle = acos(a_cos) * 180 / PI;
            if(angle >= angle_threshold){
                Segment s = Segment(*f_p,*l_p,*c_p);
                segment_set.push_back(s);
            }
            f_p = p_iter;
            l_p = p_iter;
            c_p = p_iter;
            _min = 11;
        }
    }
    double a_cos =(pow((*f_p).length(),2) + pow((*l_p).length(),2) - pow((*l_p - *f_p).length(),2))/(2.0*((*f_p).length())*((*l_p).length()));
    double angle = acos(a_cos) * 180 / PI;
    if(angle >= angle_threshold){
        Segment s = Segment(*f_p,*l_p,*c_p);
        segment_set.push_back(s);
    }
}
void lidar_obstacle::detect_circle(){
    for(auto& s: segment_set){
        // closet norm 방향 벡터 찾기
        Point norm = (-s.closest_point).normalized(); // !!주의 
        // closeset 점 원점 수선 찾기
        int m1; int b1;  // ax +by +c = 0
        m1 = (s.closest_point.x/-s.closest_point.y); // 수선 기울기
        b1 = -m1*s.closest_point.x + s.closest_point.y;
        // first 점, last 점 원점 직선 찾기
        int m2; int m3;
        m2 = s.first_point.y/s.first_point.x;
        m3 = s.last_point.y/s.last_point.x;
        Point _start; Point _end ; Point _mid;
        _start.x = b1/(m2-m1); _start.y = m2*(b1/(m2-m1));
        _end.x = b1/(m3-m1); _end.y = m3*(b1/(m3-m1));

        double radius = 0.5773502 * (_start - _end).length();  // sqrt(3)/3 * length 비율 수정 가능
        Point center = (_start + _end - radius * norm) / 2.0;
        Circle c = Circle(center, radius);
        circle_set.push_back(c);
        // Point first_point;
        // Point last_point;
        // Point closest_point;
    }



}
int main(){

}

