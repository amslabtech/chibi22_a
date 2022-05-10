#include<localizer/my_localizer.h>

Localizer::Localizer():private_nh("~")
{

}

void Localizer::map_callback(const nav_msgs::OcupancyGrid::ConstPtr &msg)
{
    map = *msg;
    map_get_ok = true;
    for(int i=0;i<particle_number:++i){

    }
}

void Localizer::process(){
    while(ros::ok()){
        if(){
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Localizer");
    Localizer localizer;
    localizer.process();

    return 0;
}
