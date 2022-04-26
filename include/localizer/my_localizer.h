#ifndef INCLUDE_my_localizer_h_ //条件コンパイル
#define INCLUDE_my_localizer_h_

#include <nav_msgs/OccupancyGrid.h>:wq

class Localizer
{
    public:
        Localizer();
        void process();
    private:
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
}

#endif
