#include<math.h>
using namespace sensor_fusion;

class Segment
{
    public:
        Point first_point;
        Point last_point;
        Point closest_point;

        Segment(const Point& p1 = Point(), const Point& p2 = Point(), const Point& p3 = Point()) {
            first_point = p1;
            last_point = p2;
            closest_point = p3;
        }







};