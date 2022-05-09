#include<math.h>
using namespace sensor_fusion;

class Circle
{
    public:
        Point center;
        double r;
      

        Circle(const Point& p1 = Point(), const double radius = 0) {
            center = p1;
            r = radius;
        }
};