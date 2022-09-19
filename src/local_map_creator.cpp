#include <ros/ros.h>
#include <local_map_creator/local_map_creator.h>

Local_map_creator::Local_map_creator():private_nh("~"){
    //paramators
    private_nh.getParam("hz", hz);
    private_nh.getParam("map_size", map_size);
    private_nh.getParam("map_reso",map_reso);
    private_nh.getParam("laser_density", laser_density);
    private_nh.getParam("roomba_radius", roomba_radius);
    private_nh.getParam("ignore_angle_mergin", ignore_angle_mergin);

    sub_laser = nh.subscribe("scan", 10, &Local_map_creator::laser_callback, this);
    pub_local_map = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 10);
    pub_obstacle_poses = nh.advertise<geometry_msgs::PoseArray>("obstacle_poses", 10);
    obstacle_poses.header.frame_id = "base_link";
    local_map.header.frame_id = "base_link";
    local_map.info.resolution = map_reso;
    local_map.info.width = map_size / map_reso;
    local_map.info.height = map_size / map_reso;
    local_map.info.origin.position.x = - map_size / 2;
    local_map.info.origin.position.y = - map_size / 2;
    local_map.data.reserve(local_map.info.width * local_map.info.height);
    init_map();
}

void Local_map_creator::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
    laser = *msg;
    init_map();
    create_local_map();
    laser_get_check = true;
}

void Local_map_creator::init_map(){         //まっさらな地図の作成
    local_map.data.clear();     //配列の初期化
    size = local_map.info.width * local_map.info.height;        //なんかけなんの地図を考える
    for(int i = 0; i < size; i++){
        local_map.data.push_back(-1);       //未知の値は-１,すべてのマスに未知の情報を挿入
    }
}

int Local_map_creator::xy_to_map_index(double x, double y){
    int index_x = int((x - local_map.info.origin.position.x) / local_map.info.resolution);
    int index_y = int((y - local_map.info.origin.position.y) / local_map.info.resolution);

    return index_x + index_y * local_map.info.width;
}

bool Local_map_creator::check_map_range(double x, double y){        //地図の範囲内に収まっているかどうかチェック
    double x_start = local_map.info.origin.position.x;      //最初の位置x
    double y_start = local_map.info.origin.position.y;      //最初の位置y
    double x_end = x_start + local_map.info.width * local_map.info.resolution;
    double y_end = y_start + local_map.info.height * local_map.info.resolution;

    if(x_start < x && x < x_end && y_start < y && y < y_end){   //x, yが最初と最後の範囲に収まっているかどうか
        return true;
    } else {
        return false;
    }
}

bool Local_map_creator::is_ignore_angle(double angle){      //建物の柱を無視する、以下の条件のときは全部false
    if(angle > -3.0/4 * M_PI + ignore_angle_mergin && angle < -1.0/4 * M_PI - ignore_angle_mergin){     //角度で判定
       return false;
   } else if(angle > -1.0/4 * M_PI + ignore_angle_mergin && angle < 1.0/4 * M_PI - ignore_angle_mergin){
       return false;
   }else if(angle > 1.0/4 * M_PI + ignore_angle_mergin && angle < 3.0/4 * M_PI - ignore_angle_mergin){
       return false;
   }
   return true;
}

void Local_map_creator::create_line(double yaw, double laser_range){    //受け取ったレーザーの値をinit mapに書き込んでいくコード
    double search_step = map_reso;
    if(laser_range <= roomba_radius || is_ignore_angle(yaw)){
        laser_range = map_size;
    }
    for(double dist_from_start=0; dist_from_start < map_size; dist_from_start += search_step){  //正面の地図はすべて-1、そこに走れるところは0を、壁などの障害物には100
        double x_now = dist_from_start * std::cos(yaw);     //yawはroomba座標系から見た角度
        double y_now = dist_from_start * std::sin(yaw);
        if(!check_map_range(x_now, y_now)){return;}
        int map_index = xy_to_map_index(x_now, y_now);
        if(dist_from_start >= laser_range){
            local_map.data[map_index] = 100;
            geometry_msgs::Pose obstacle_pose;
            obstacle_pose.position.x = x_now;
            obstacle_pose.position.y = y_now;
            obstacle_poses.poses.push_back(obstacle_pose);
            return;
        } else {
            local_map.data[map_index] = 0;
        }
    }
}

void Local_map_creator::create_local_map(){     //create lineは一本のせんなのでそれを張り巡らせることで地図を作る。
    obstacle_poses.poses.clear();
    double scan_angle = laser.angle_max - laser.angle_min;

    //1080本のレーザーが出てるけど全部使うと処理が重い。実際にはレーザーの精度は5㎝くらいだから無駄になるレーザーの本数を最初から減らしておくことで
    //処理を軽くすることができる。
    //laser_densityはレーザーの密度に関する変数。
    int laser_step = int(2 * map_reso * laser.ranges.size() / laser_density / scan_angle / map_size);
    for(int i=0; i<int(laser.ranges.size()); i+=laser_step){
        double angle = i * laser.angle_increment + laser.angle_min;
        create_line(angle, laser.ranges[i]);
    }
}

void Local_map_creator::process(){
    ros::Rate rate(hz);
    while(ros::ok()){
        if(laser_get_check){
            pub_local_map.publish(local_map);
            pub_obstacle_poses.publish(obstacle_poses);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "local_map_creator");
    Local_map_creator local_map_creator;
    local_map_creator.process();

    return 0;
}
