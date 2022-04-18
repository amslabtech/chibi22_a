#include <local_map_creator/local_map_creator.h>

Local_map_creator::Local_map_creator():private("~"){
    //paramators
    pravate_nh.getParam("hz", hz);
    private_nh.getParam("map_size", map_size);
    private_nh.getParam("map_reso",map_reso);
    private_nh.getParam("laser_density", laser_density);

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

    return index_x * index_y * local_map.info.width;
}

bool local_map_creator::check_map_range(double x, double y){        //地図の範囲内に収まっているかどうかチェック
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

bool Local_map_creator::is_ignore_angle(){
    if(angle > -3.0/4 * M_PI + ignore_angle_mergin && angle < -1.0/4 * M_PI - ignore_angle_mergin){
       return false;
   } else if(angle > -1.0/4 * M_PI + ignore_angle_mergin && angle < 1.0/4 * M_PI - ignore_angle_mergin){
       return false;
   }else if(angle > 1.0/4 * M_PI + ignore_angle_mergin && angle < 3.0/4 * M_PI - ignore_angle_mergin){
       return false;
   }
   return true;
}

void Local_map_creator::create_line(double yaw, double laser_range){
    double serch_step = map_reso;
    if(laser_range <= roomba_radius || is_ignore_angle(yaw)){
        laser_range = map_size;
    }
}