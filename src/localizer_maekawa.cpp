#include <localizer/localizer_maekawa.h>
#include <tf2_ros/transform_broadcaster.h>
std::random_device seed;
std::mt19937 engine(seed());
std::default_random_engine engine2(seed());


Localizer::Localizer():Localizer_nh("~")
{
    Localizer_nh.getParam("hz", hz);
    Localizer_nh.getParam("particle_number", particle_number);
    Localizer_nh.getParam("initial_x", initial_x);
    Localizer_nh.getParam("initial_y", initial_y);
    Localizer_nh.getParam("initial_yaw", initial_yaw);
    Localizer_nh.getParam("initial_x_sigma", initial_x_sigma);
    Localizer_nh.getParam("initial_y_sigma", initial_y_sigma);
    Localizer_nh.getParam("initial_yaw_sigma", initial_yaw_sigma);
    Localizer_nh.getParam("move_noise_ratio", move_noise_ratio);
    Localizer_nh.getParam("laser_noise_ratio", move_noise_ratio);
    Localizer_nh.getParam("search_range", search_range)
    Localizer_nh.getParam("laser_step", laser_step);
    Localizer_nh.getParam("alpha_slow_th", alpha_slow_th);
    Localizer_nh.getParam("alpha_fast_th", alpha_fast_th);


    map_sub=nh.subscribe("/map", 1, &Localizer::map_callback, this);

    p_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/p_pose_array", 10);

    laser_sub = nh.subscribe("/scan", 10, &Localizer::laser_callback, this);
    odometry_sub = nh.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    estimated_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 10);

    estimated_pose.pose.position.x = 0.0;
    estimated_pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), estimated_pose.pose.orientation);

    estimated_pose.header.frame_id = "map";
    p_pose_array.header.frame_id="map";
    p_pose_array.poses.reserve(particle_number);
    p_array.reserve(particle_number);
}

Localizer::Particle::Particle(Localizer* localizer)
{
    mcl=localizer;
    p_pose.header.frame_id="map";
    set_p(mcl->initial_x, mcl->initial_y, mcl->initial_yaw, mcl->initial_x_sigma, mcl->initial_y_sigma, mcl->initial_yaw_sigma);
    w = 1.0/mcl->particle_number;
}

double Localizer::gaussian(double mu, double sigma)
{
    std::normal_distribution<> dist(mu, sigma);
    return dist(engine);
}

double Localizer::adjust_yaw(double yaw)
{
    if(yaw>M_PI)
    {
        yaw-=2*M_PI;
    } else if(yaw<-M_PI){
        yaw+=2*M_PI;
    }
    return yaw;
}


void Localizer::create_p_pose_array_from_p_array(std::vector<geometry_msgs::PoseStamped> &p_array)
{
    p_pose_array.poses.clear();
    p_pose_array.header.frame_id = "map";
    for(auto& p:p_array)
    {
        p_pose_array.poses.push_back(p.pose);
    }
}

void Localizer::set_particle(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
{
    p_pose.pose.position.x = gaussian(x, x_sigma);
    p_pose.pose.position.y = gaussian(y, y_sigma);
    yaw = adjust_yaw(gaussian(yaw, yaw_sigma));
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), p_pose.pose.orientation);
}


void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map=*msg;
    map_get_ok=true;
    for(int i=0; i<particle_numberl i++){
        geometry_msgs::PoseStamped p;
        p_array.push_back(p);
    }
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
     laser = *msg;
     if(map_get_ok){
         observation_update();
     }
 }

 void Localizer::observation_update()
 {
     for(auto& p:p_array)
     {
         double weight = calc_w(p.p_pose);
         p.w=weight;
     }
     estimate_pose();
     double estimated_pose_w=calc_w(estimated_pose)/(laser.range.size()/laser_step);
     if(alpha_slow==0)
     {
         alpha_slow=alpha;
     }else{
         alpha_slow += alpha_slow_th*(alpha-alpha_slow);
     }
     if(alpha_fast==0)
     {
         alpha_fast=alpha;
     }else{
         alpha_fast+=alpha_fast_th*(alpha-alpha_fast);
     }
     if(estimated_pose_w > estimated_pose_w_th||reset_count>reset_limit)
     {
         reset_count=0;
         adaptive_resampling();
     } else{
         reset_count+=1;
         expansion_reset();
     }
 }

}

void Localizer::estimate_pose()
{
    normarize_w();
    double x,y,yaw,w_max=0;
    for(auto& p:p_array){
        x+=p.p_pose.pose.position.x * p.w;
        y+=p.p_pose.pose.position.y * p.w;
        if(

void Localizer::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(map_get_ok)
        {
            std::cout<<"map_get_ok"<<std::endl;
            create_p_pose_array_from_p_array(p_array);
            std::cout<<"p_pose_array number:"<<p_pose_array.poses.size()<<std::endl;
            std::cout<<"p_array number:"<<p_array.size()<<std::endl;
            std::cout<<"particle number:"<<particle_number<<std::endl;
            p_pose_array_pub.publish(p_pose_array);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Localizer");
    Localizer localizer;
    localizer.process();



    return 0;
}

