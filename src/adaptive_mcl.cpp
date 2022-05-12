#include<localizer/localizer.h>
std::random_device seed;
std::mt19937 engine(seed());
std::default_random_engine engine2(seed());

Localizer::Localizer():private_nh("~")
{
    private_nh.getParam("hz", hz);
    // map_callback
    private_nh.getParam("particle_number", particle_number);
    private_nh.getParam("init_x", init_x);
    private_nh.getParam("init_y", init_y);
    private_nh.getParam("init_yaw", init_yaw);
    private_nh.getParam("init_x_sigma", init_x_sigma);
    private_nh.getParam("init_y_sigma", init_y_sigma);
    private_nh.getParam("init_yaw_sigma", init_yaw_sigma);
    private_nh.getParam("intercept", intercept);
    private_nh.getParam("random_noise", random_noise);
    private_nh.getParam("delta", delta);
    private_nh.getParam("epsilon", epsilon);

    // odometry_callback
    private_nh.getParam("move_noise_ratio", move_noise_ratio);

    // laser_callback
    private_nh.getParam("laser_noise_ratio", laser_noise_ratio);
    private_nh.getParam("laser_step", laser_step);
    private_nh.getParam("alpha_slow_th", alpha_slow_th);
    private_nh.getParam("alpha_fast_th", alpha_fast_th);
    private_nh.getParam("estimated_pose_w_th", estimated_pose_w_th);
    private_nh.getParam("reset_limit", reset_limit);
    private_nh.getParam("reset_x_sigma", reset_x_sigma);
    private_nh.getParam("reset_y_sigma", reset_y_sigma);
    private_nh.getParam("reset_yaw_sigma", reset_yaw_sigma);
    private_nh.getParam("expansion_x_speed", expansion_x_speed);
    private_nh.getParam("expansion_y_speed", expansion_y_speed);
    private_nh.getParam("expansion_yaw_speed", expansion_yaw_speed);
    private_nh.getParam("search_range", search_range);

    private_nh.getParam("kld_switch", kld_switch);
    private_nh.getParam("aug_switch", aug_switch);

    // sub
    map_sub = nh.subscribe("/map", 1, &Localizer::map_callback, this);
    odometry_sub = nh.subscribe("/roomba/odometry", 1, &Localizer::odometry_callback, this);
    laser_sub = nh.subscribe("/scan", 1, &Localizer::laser_callback, this);

    // map_callback
    p_pose_array.header.frame_id = "map";
    p_array.reserve(particle_number);
    p_pose_array.poses.reserve(particle_number);

    // laser_callback
    estimated_pose.header.frame_id = "map";
    estimated_pose.pose.position.x = 0.0;
    estimated_pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), estimated_pose.pose.orientation);

    // pub
    p_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/p_pose_array", 1);
    estimated_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
}

// Gaussian 乱数生成
double Localizer::gaussian(double mu, double sigma)
{
        std::normal_distribution<> dist(mu, sigma);
        return dist(engine);
}

// Gaussian
double Localizer::gaussian(double mu, double sigma, double x)
{
        double ans = exp(- std::pow((mu - x), 2) / std::pow(sigma, 2) / 2.0) / sqrt(2.0 * M_PI * std::pow(sigma, 2));
        return ans;
}

//liner
double Localizer::liner(double x, double sigma)
{
    double slope = -intercept / (5 * sigma);
    return slope * x + intercept;
}

//random_noise
double Localizer::w_noise(double mu, double sigma, double x)
{
    double ans = gaussian(mu, sigma, x);
    double liner_ans = liner(x, sigma);
    if(liner_ans > ans){ans = liner_ans;}
    if(random_noise > ans){ans = random_noise;}
    return ans;
}

double Localizer::chi2(double x, double k)
{
    double ans = (1 / (std::pow(2, k/2.0) * std::tgamma(k/2.0))) * std::pow(x, (k/2.0)-1) * exp(-x/2.0);
    return ans;
}

double Localizer::y(double k)
{
    double x = 0;
    double dx = 0.01;
    double sum = 0;
    while(sum <= 1.0-delta){
        sum += chi2(x, k);
        // std::cout << "sum:" << sum << std::endl;
        x += dx;
    }
    // std::cout << "x:" << x << std::endl;
    return x;
}

int Localizer::kld(double binnum)
{
    if(binnum <= 1){
        binnum = 2;
    }
    int p_num = std::ceil(y(binnum-1) / (2*epsilon));
    // int p_num = (int)(chi2(1-delta, binnum-1) / (2*epsilon));
    return p_num;
}

// -pie < yaw < pie
double  Localizer::adjust_yaw(double yaw)
{
    if(yaw > M_PI){yaw -= 2*M_PI;}
    if(yaw < -M_PI){yaw += 2*M_PI;}

    return yaw;
}

// ノイズ付与
void Localizer::Particle::set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
{
    p_pose.pose.position.x = mcl->gaussian(x, x_sigma);
    p_pose.pose.position.y = mcl->gaussian(y, y_sigma);
    yaw = mcl->adjust_yaw(mcl->gaussian(yaw, yaw_sigma));
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw),p_pose.pose.orientation);
}

// 初期化
Localizer::Particle::Particle(Localizer *localizer)
{
    mcl = localizer;
    p_pose.header.frame_id = "map";
    set_p(mcl->init_x, mcl->init_y, mcl->init_yaw, mcl->init_x_sigma, mcl->init_y_sigma, mcl->init_yaw_sigma);
    w = 1.0 / mcl->particle_number;
}

// 親クラスにアクセス
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

// ノイズ付与
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

void Localizer::kld_resampling(double dtrans, double drot1, double drot2)
{
    std::uniform_real_distribution<> random(0.0, 1.0);
    double r = random(engine2);
    int index = 0;
    std::vector<Particle> tmp_p_array;
    std::set<int> bin_list = {};
    int p_num = p_array.size();
    while(p_num>=tmp_p_array.size() && tmp_p_array.size()<particle_number){
        r += 1.0 / p_array.size();
        while(r > p_array[index].w){
            r -= p_array[index].w;
            index = (index + 1) % p_array.size();
        }

        Particle p = p_array[index];
        p.p_move(dtrans, drot1, drot2);
        double x = p.p_pose.pose.position.x;
        double y = p.p_pose.pose.position.y;
        int map_index = xy_to_map_index(x, y);

        bin_list.insert(map_index);
        double binnum = bin_list.size();
        p_num = kld(binnum);
        // std::cout << "binnum:" << binnum << std::endl;
        // std::cout << "p_num:" << p_num << std::endl;
        tmp_p_array.push_back(p);
        // std::cout << tmp_p_array.size() << std::endl;
    }
    p_array = tmp_p_array;
    // std::cout << tmp_p_array.size() << std::endl;
}

// 動作更新
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

    if(kld_switch){
        kld_resampling(dtrans, drot1, drot2);
    }else{
        for(auto &p:p_array){
            p.p_move(dtrans, drot1, drot2);
        }
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

int Localizer::xy_to_map_index(double x, double y)
{
    int index_x = int((x - map.info.origin.position.x) / map.info.resolution);
    int index_y = int((y - map.info.origin.position.y) / map.info.resolution);
    return index_x + index_y * map.info.width;
}

double Localizer::dist_from_p_to_wall(double x_start, double y_start, double yaw, double laser_range)
{
    double search_step = map.info.resolution;
    // double search_limit = std::min(laser_range * (1.0 + laser_noise_ratio * 5), search_range);
    double search_limit = laser_range * (1.0 + laser_noise_ratio * 5) * 2;
    for(double dist_from_start = search_step; dist_from_start <= search_limit; dist_from_start += search_step){
        double x_now = x_start + dist_from_start * cos(yaw);
        double y_now = y_start + dist_from_start * sin(yaw);
        int map_index = xy_to_map_index(x_now, y_now);
        if(map.data[map_index] == 100){
            return dist_from_start;
        }
        if(map.data[map_index] == -1){
            return search_limit;
        }
    }
    return search_limit;
}

// 重み計算
double Localizer::calc_w(geometry_msgs::PoseStamped &pose)
{
    double weight = 0;
    double angle_increment = laser.angle_increment;
    double angle_min = laser.angle_min;
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = tf::getYaw(pose.pose.orientation);
    for(int i=0, size=laser.ranges.size(); i<size; i+=laser_step){
        if(laser.ranges[i] > ignore_laser){
            double angle = i * angle_increment + angle_min;
            double dist_to_wall = dist_from_p_to_wall(x, y, yaw + angle, laser.ranges[i]);
            weight += w_noise(laser.ranges[i], laser.ranges[i] * laser_noise_ratio, dist_to_wall);
        }
    }
    return weight;
}

// 重みの正規化
void Localizer::normalize_w()
{
    alpha = 0;
    for(auto &p:p_array){
        alpha += p.w;
    }
    for(auto &p:p_array){
        p.w /= alpha;
    }
}

// 位置推定
void Localizer::estimate_pose()
{
    normalize_w();
    double x = 0;
    double y = 0;
    double yaw = 0;
    double w_max = 0;
    for(auto &p:p_array){
        // x += p.p_pose.pose.position.x * p.w;
        // y += p.p_pose.pose.position.y * p.w;
        if(p.w > w_max){
            w_max = p.w;
            x = p.p_pose.pose.position.x;
            y = p.p_pose.pose.position.y;
            yaw = tf::getYaw(p.p_pose.pose.orientation);
        }
    }

    estimated_pose.pose.position.x = x;
    estimated_pose.pose.position.y = y;
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), estimated_pose.pose.orientation);
}

void Localizer::augmented_resampling()
{
    std::uniform_real_distribution<> random(0.0, 1.0);

    double r = random(engine2);
    int index = 0;
    double reset_ratio = 1 - (alpha_fast / alpha_slow);
    std::vector<Particle> p_array_after_resampling;
    p_array_after_resampling.reserve(p_array.size());
    double reset_coef = std::max(0.0, reset_ratio);
    int reset_quantity = 0;
    if(aug_switch){int reset_quantity = (int)(reset_coef * p_array.size());}
    for(int i=0, size=p_array.size() - reset_quantity; i<size; ++i){
        r += 1.0 / p_array.size();
        while(r > p_array[index].w){
            r -= p_array[index].w;
            index = (index + 1) % p_array.size();
        }
        Particle p = p_array[index];
        p.w = 1.0 / particle_number;
        p_array_after_resampling.push_back(p);
    }

    for(int i=0, size=reset_quantity; i<size; i++){

        Particle p = p_array[0];
        p.w = 1.0 / p_array.size();

        double x = estimated_pose.pose.position.x;
        double y = estimated_pose.pose.position.y;
        double yaw = tf::getYaw(estimated_pose.pose.orientation);

        p.set_p(x, y, yaw, reset_x_sigma, reset_y_sigma, reset_yaw_sigma);

        p_array_after_resampling.push_back(p);
    }
    p_array = p_array_after_resampling;
}

// 膨張リセット
void Localizer::expansion_reset()
{
    for(auto &p:p_array){
        double x = p.p_pose.pose.position.x;
        double y = p.p_pose.pose.position.y;
        double yaw = tf::getYaw(p.p_pose.pose.orientation);

        p.set_p(x, y, yaw, expansion_x_speed, expansion_y_speed, expansion_yaw_speed);
    }
}

void Localizer::calc_alphas()
{
    double alpha_ave = alpha / particle_number;
    if(alpha_slow == 0){
        alpha_slow = alpha_ave;
    }
    else{
        alpha_slow += alpha_slow_th * (alpha_ave - alpha_slow);
    }

    if(alpha_fast == 0){
        alpha_fast = alpha_ave;
    }
    else{
        alpha_fast += alpha_fast_th * (alpha_ave - alpha_fast);
    }
}

// 観測更新
void Localizer::observation_update()
{
    for(auto &p:p_array){
        double weight = calc_w(p.p_pose);
        p.w = weight;
    }
    estimate_pose();
    double estimated_pose_w = calc_w(estimated_pose) / (laser.ranges.size() / laser_step);
    calc_alphas();

    if(estimated_pose_w > estimated_pose_w_th || reset_count > (p_array.size() * reset_limit)){
        reset_count = 0;
        augmented_resampling();
    }
    else{
        reset_count += 1;
        expansion_reset();
    }
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser = *msg;
    if(map_get_ok){
        observation_update();
    }
}

// p_pose_arrayにp_pose.pose
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
    tf2_ros::TransformBroadcaster odom_state_broadcaster;
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(map_get_ok && odometry_get_ok){
            try{
                double map2base_x = estimated_pose.pose.position.x;
                double map2base_y = estimated_pose.pose.position.y;
                double map2base_yaw = tf::getYaw(estimated_pose.pose.orientation);

                double odom2base_x = current_odometry.pose.pose.position.x;
                double odom2base_y = current_odometry.pose.pose.position.y;
                double odom2base_yaw = tf::getYaw(current_odometry.pose.pose.orientation);

                double map2odom_yaw = adjust_yaw(map2base_yaw - odom2base_yaw);
                double map2odom_x = map2base_x - odom2base_x * cos(map2odom_yaw) + odom2base_y * sin(map2odom_yaw);
                double map2odom_y = map2base_y - odom2base_x * sin(map2odom_yaw) - odom2base_y * cos(map2odom_yaw);
                geometry_msgs::Quaternion map2odom_quat;
                quaternionTFToMsg(tf::createQuaternionFromYaw(map2odom_yaw), map2odom_quat);

                geometry_msgs::TransformStamped odom_state;
                odom_state.header.stamp = ros::Time::now();
                odom_state.header.frame_id = "map";
                odom_state.child_frame_id = "odom";
                odom_state.transform.translation.x = map2odom_x;
                odom_state.transform.translation.y = map2odom_y;
                odom_state.transform.rotation = map2odom_quat;

                odom_state_broadcaster.sendTransform(odom_state);


            }catch(tf2::TransformException &ex){
                ROS_ERROR("%s", ex.what());
            }
            create_p_pose_array_from_p_array(p_array);
            p_pose_array_pub.publish(p_pose_array);
            estimated_pose_pub.publish(estimated_pose);
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
