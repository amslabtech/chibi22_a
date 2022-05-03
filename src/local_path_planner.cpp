#include "chibi22_a/local_path_planner.h"

DynamicWindowApproach::DynamicWindowApproach():private_nh("~")
{
    private_nh.param("max_speed",max_speed,{1.0});
    private_nh.param("min_speed",min_speed,{0.0});
    private_nh.param("max_yawrate",max_yawrate,{0.8});
    private_nh.param("max_accel",max_accel,{1000.0});
    private_nh.param("max_dyawrate",max_dyawrate,{2.0});
    private_nh.param("dt",dt,{0.5});
    private_nh.param("predict_time",predict_time,{3.0});
    private_nh.param("gain_heading",gain_heading,{1.0});
    private_nh.param("gain_velocity",gain_velocity,{1.0});
    private_nh.param("gain_obs",gain_obs,{1.0});
    private_nh.param("world",world,{4.0});
    private_nh.param("safemass_x",safemass_x,{10});
    private_nh.param("safemass_y",safemass_y,{10});
    private_nh.param("distance",distance,{0.0});
    private_nh.param("dist_min",dist_min,{0.0});
    private_nh.param("max_cost",max_cost,{0.0});
    private_nh.param("v_reso",v_reso,{0.1});
    private_nh.param("omega_reso",omega_reso,{0.1});
    private_nh.param("current_velocity",current_velocity,{0.0});
    private_nh.param("current_omega",current_omega,{0.0});
    private_nh.param("column",column,{0});
    private_nh.param("row",row,{0});
    private_nh.param("resolution",resolution,{0.0});
    private_nh.param("goal_tolerance",goal_tolerance,{0.3});
    private_nh.param("wait",wait,{0.0});
    private_nh.param("hz",hz,{10});

    sub_local_goal = nh.subscribe("local_goal",10,&DynamicWindowApproach::local_goal_callback,this);
    sub_pose = nh.subscribe("pose",10,&DynamicWindowApproach::pose_callback,this);
    sub_local_map = nh.subscribe("local_map",10,&DynamicWindowApproach::local_map_callback,this);

    pub_predict_path = nh.advertise<nav_msgs::Path>("predict_path",1);
    pub_best_predict_path = nh.advertise<nav_msgs::Path>("best_predict_path",1);
    pub_roomba_ctrl = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
}

void DynamicWindowApproach::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_goal=*msg;
}

void DynamicWindowApproach::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose=*msg;
}

void DynamicWindowApproach::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    local_map = *msg;
    if(local_map.data.size()!=0)set_map();
}

void DynamicWindowApproach::set_map()
{
    column = local_map.info.width;
    row = local_map.info.height;
    resolution = local_map.info.resolution;
    map.resize(row,std::vector<int>(column));

    for(int i=0;i<row;i++)
    {
        for(int j=0;j<column;j++)
        {
            map[i][j] = local_map.data[j*row+i];
        }
    }
    receive_local_map = true;
}

bool DynamicWindowApproach::goal_reach()
{
    double dx = pose.pose.position.x - local_goal.pose.position.x;
    double dy = pose.pose.position.y - local_goal.pose.position.y;
    double dist_goal = sqrt(std::pow(dx,2)+std::pow(dy,2));

    if(dist_goal < goal_tolerance )
    {
        return true;
    }
    else
    {
        return false;
    }
}

//ダイナミックウィンドウを作成
void DynamicWindowApproach::create_dynamic_window()
{
    std::vector<double> Vs = {max_speed,min_speed,max_yawrate,-1*max_yawrate};

    std::vector<double> Vd = {
        current_velocity + max_accel*dt,
        current_velocity - max_accel*dt,
        current_omega + max_dyawrate,
        current_omega - max_dyawrate
    };

    dw.max_v = std::min(Vs[0],Vd[0]);
    dw.min_v = std::max(Vs[1],Vd[1]);
    dw.max_yawrate = std::min(Vs[2],Vd[2]);
    dw.min_yawrate = std::max(Vs[3],Vd[3]);

}

//軌跡を計算
void DynamicWindowApproach::calc_trajectory(const double &v, const double &omega)
{
    traj.clear();

    predict_path.poses.clear();
    roomba_traj = {0.0,0.0,0.0,0.0,0.0};

    for(double time=0; time<=predict_time;time+=dt)
    {
        roomba_traj.yaw += omega * dt;
        roomba_traj.x += v * std::sin(roomba_traj.yaw) * dt;
        roomba_traj.y += v * std::cos(roomba_traj.yaw) * dt;
        roomba_traj.v = v;
        roomba_traj.omega = omega;
        traj.push_back(roomba_traj);

        geometry_msgs::PoseStamped sim_pose;
        sim_pose.pose.position.x = roomba_traj.x;
        sim_pose.pose.position.y = roomba_traj.y;
        predict_path.poses.push_back(sim_pose);
    }
    predict_path.header.frame_id= "base_link";
    pub_predict_path.publish(predict_path);
}

//評価関数の計算
double DynamicWindowApproach::calc_evaluation()
{
    double cost_heading = calc_cost_heading() * gain_heading;
    double cost_velocity = calc_cost_velocity() * gain_velocity;
    //std::cout<<"obs"<<std::endl;
    double cost_obs = calc_cost_obstacle() * gain_obs;
    //std::cout<<"obs"<<std::endl;
    // std::cout<<"heading,velocity,obstacle:"<<cost_heading<<","<<cost_velocity<<","<<cost_obs<<std::endl;

    double cost_total = cost_heading + cost_velocity + cost_obs;

    return cost_total;
}

//角度の評価関数
double DynamicWindowApproach::calc_cost_heading()
{
    double goal_theta = std::atan2(local_goal.pose.position.y - pose.pose.position.y - traj.back().y,local_goal.pose.position.x - pose.pose.position.x - traj.back().x);
    double score_angle = goal_theta - tf::getYaw(pose.pose.orientation) - traj.back().yaw;
    if(score_angle >  M_PI) score_angle -= 2*M_PI;
    if(score_angle < -M_PI) score_angle += 2*M_PI;

    // std::cout<<std::abs(score_angle)/M_PI<<std::endl;
    return std::abs(score_angle)/M_PI;
}

//速度の評価関数
double DynamicWindowApproach::calc_cost_velocity()
{
    // std::cout<<"velocity:"<<(max_speed-traj.back().v)/max_speed<<std::endl;
    return (max_speed - traj.back().v)/max_speed;
}

//障害物の評価関数
double DynamicWindowApproach::calc_cost_obstacle()
{
    dist_min = 1e3;
    max_cost = 0.0;

    for(auto& state : traj)
    {
        int x = (int)((state.x - (-world/2))/resolution);
        int y = (int)((state.y - (-world/2))/resolution);

        for(int i = x-safemass_x; i<x+safemass_x; i++)
        {
            for(int j = y-safemass_y; j<y+safemass_y; j++)
            {
                if(i>0 && j>0 && i<row && j<column && (map[i][j] == 100))
                {
                    a = double(-world/2 + i*resolution);
                    b = double(-world/2 + j*resolution);

                    distance = sqrt(std::pow(state.x-a,2)+std::pow(state.y-b,2));
                    if(b<0)
                    {
                        continue;
                    }

                    if(distance <= 0.2)
                    {
                        // distance = sqrt(std::pow(state.x-a,2)+std::pow(state.y-b,2));
                        max_cost = 1e10;
                    }
                    if(dist_min >= distance)
                    {
                        dist_min = distance;
                    }
                }
            }
        }
    }
    double dist_max = sqrt(pow(safemass_x,2)+pow(safemass_y,2));
    // std::cout<<"max_cost:"<<max_cost<<","<<"1/dist_min"<<1.0/dist_min<<std::endl;
    if(max_cost>dist_min) return max_cost;
    else return 1.0-(dist_min/dist_max);
}

void DynamicWindowApproach::calc_final_input()
{
    min_cost = 1e10;
    best_traj.clear();
    final_cost = 0.0;

    //std::cout<<"1"<<std::endl;

    for(double v = dw.min_v; v<=dw.max_v; v+=v_reso )
    {
        for(double omega = dw.min_yawrate; omega<=dw.max_yawrate; omega+=omega_reso)
        {
            calc_trajectory(v,omega);
            //std::cout<<"2"<<std::endl;
            final_cost = calc_evaluation();
            // std::cout<<final_cost<<std::endl;
            //std::cout << "3"<<std::endl;
            if(min_cost >= final_cost)
            {
                //std::cout<<"4"<<std::endl;
                min_cost = final_cost;
                min_v = v;
                min_yawrate = omega;
                best_traj = traj;
                //std::cout<<"4"<<std::endl;
            }
            current_velocity = min_v;
            current_omega = min_yawrate;
            //std::cout<<"5"<<std::endl;
        }
    }
    std::cout<<"min_v:"<<min_v<<"min_yawrate"<<min_yawrate<<std::endl;
    visualize_traj();
    //std::cout<<"6"<<std::endl;
}

void DynamicWindowApproach::visualize_traj()
{
    best_predict_path.poses.clear();
    for(auto& state : best_traj)
    {
        geometry_msgs::PoseStamped best_path_point;
        best_path_point.pose.position.x = state.x;
        best_path_point.pose.position.y = state.y;
        best_predict_path.poses.push_back(best_path_point);
    }
    best_predict_path.header.frame_id = "base_link";
    pub_best_predict_path.publish(best_predict_path);
}

void DynamicWindowApproach::roomba_control(double v,double yawrate)
{
    cmd_vel.mode=11;
    cmd_vel.cntl.linear.x = v;
    cmd_vel.cntl.angular.z = yawrate;
    pub_roomba_ctrl.publish(cmd_vel);
}

void DynamicWindowApproach::dwa_control()
{
    create_dynamic_window();
    //std::cout <<"win" << std::endl;
    calc_final_input();
    //std::cout <<"input"<< std::endl;

    if(wait < 5)
    {
        roomba_control(0.0,0.0);
        wait++;
        //std::cout <<"w"<<std::endl;
    }
    else
    {
        roomba_control(min_v,min_yawrate);
        //std::cout <<"c"<<std::endl;
    }

    if(goal_reach())
    {
        //std::cout << "goal" << std::endl;
        roomba_control(0.0,0.0);
    }

    if(dist_min == 1e10){
        roomba_control(0.0,0.0);
        //std::cout << "d"<<std::endl;
    }
}

void DynamicWindowApproach::process()
{
    ros::Rate loop_rate(hz);
    while (ros::ok())
    {
        if(receive_local_map)
        {
            dwa_control();
            receive_local_map = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv, "local_path_planner");
    DynamicWindowApproach DWA;
    DWA.process();
    return 0;
}

