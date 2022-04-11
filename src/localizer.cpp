#include<localizer/localizer.h>
std::random_device seed;
std::mt19937 engine(seed());

Localizer::Localizer():private_nh("~")
{
    private_nh.getParam("particle_number", particle_number);
    private_nh.getParam("init_x", init_x);
    private_nh.getParam("init_y", init_y);
    private_nh.getParam("init_yaw", init_yaw);
    private_nh.getParam("init_x_sigma", init_x_sigma);
    private_nh.getParam("init_y_sigma", init_y_sigma);
    private_nh.getParam("init_yaw_sigma", init_yaw_sigma);
    private_nh.getParam("move_noise_ratio", move_noise_ratio);
    map_sub = nh.subscribe("/map", 1, &Localizer::map_callback, this);
    odometry_sub = nh.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    p_pose_array.header.frame_id = "map";
    p_array.reserve(particle_number);
    p_pose_array.poses.reserve(particle_number);

    p_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/p_pose_array", 10);
}

double Localizer::gaussian(double mu, double sigma)
{
        std::normal_distribution<> dist(mu, sigma);
        return dist(engine);
}

double Localizer::gaussian(double mu, double sigma, double x)
{
        double ans = exp(- std::pow((mu - x), 2) / std::pow(sigma, 2) / 2.0) / sqrt(2.0 * M_PI * std::pow(sigma, 2));
        return ans;
}

double  Localizer::adjust_yaw(double yaw)
{
        if(yaw > M_PI){yaw -= 2*M_PI;}
        if(yaw < -M_PI){yaw += 2*M_PI;}

        return yaw;
}

void Localizer::Particle::set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
{
    p_pose.pose.position.x = mcl->gaussian(x, x_sigma);
    p_pose.pose.position.y = mcl->gaussian(y, y_sigma);
    yaw = mcl->adjust_yaw(mcl->gaussian(yaw, yaw_sigma));
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw),p_pose.pose.orientation);
}

Localizer::Particle::Particle(Localizer *localizer)
{
    mcl = localizer;
    p_pose.header.frame_id = "map";
    set_p(mcl->init_x, mcl->init_y, mcl->init_yaw, mcl->init_x_sigma, mcl->init_y_sigma, mcl->init_yaw_sigma);
    w = 1.0 / mcl->particle_number;
}

Localizer::Particle Localizer::make_particle()
{
    Particle p(this);
    return p;
}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
    map_get_ok = true;
    for(int i=0; i<particle_number; ++i){
        Particle p = make_particle();
        p_array.push_back(p);
    }
}

void Localizer::Particle::p_move(double dtrans, double drot1, double drot2)
{
    dtrans += mcl->gaussian(0.0, dtrans * mcl->move_noise_ratio);
    drot1 += mcl->gaussian(0.0, drot1 * mcl->move_noise_ratio);
    drot2 += mcl->gaussian(0.0, drot2 * mcl->move_noise_ratio);

    double yaw = tf::getYaw(p_pose.pose.orientation);
    p_pose.pose.position.x += dtrans * cos(mcl->adjust_yaw(yaw + drot1));
    p_pose.pose.position.y += dtrans * sin(mcl->adjust_yaw(yaw + drot1));
    quaternionTFToMsg(tf::createQuaternionFromYaw(mcl->adjust_yaw(yaw + drot1 + drot2)), p_pose.pose.orientation);

}

void Localizer::motion_update()
{
    double dx = current_odometry.pose.pose.position.x - previous_odometry.pose.pose.position.x;
    double dy = current_odometry.pose.pose.position.y - previous_odometry.pose.pose.position.y;
    double current_yaw = tf::getYaw(current_odometry.pose.pose.orientation);
    double previous_yaw = tf::getYaw(previous_odometry.pose.pose.orientation);
    double dyaw = adjust_yaw(current_yaw - previous_yaw);
    double dtrans = sqrt(dx*dx + dy*dy);
    double drot1 = adjust_yaw(atan2(dy, dx) - previous_yaw);
    double drot2 = adjust_yaw(dyaw - drot1);


    for(auto& p:p_array){
        p.p_move(dtrans, drot1, drot2);
    }
}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(map_get_ok){
        previous_odometry = current_odometry;
        current_odometry = *msg;
        if(!odometry_get_ok){previous_odometry = current_odometry;}
        motion_update();
        odometry_get_ok = true;
    }
}


void Localizer::create_p_pose_array_from_p_array(std::vector<Particle> &p_array)
{
    p_pose_array.poses.clear();
    p_pose_array.header.frame_id = "map";
    for(const auto &p:p_array){
        p_pose_array.poses.push_back(p.p_pose.pose);
    }
}



void Localizer::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(map_get_ok){
            create_p_pose_array_from_p_array(p_array);
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
