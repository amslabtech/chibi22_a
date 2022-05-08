#include <local_goal_creator/local_goal_creator.h>

LocalGoalCreator::LocalGoalCreator():private_nh("~"){
    //paramator
    private_nh.getParam("hz", hz);
    private_nh.getParam("local_goal_dist", local_goal_dist);

    //subscriber, publisher
    sub_global_path = nh.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);
    sub_estimated_pose = nh.subscribe("/estimated_pose", 10, &LocalGoalCreator::estimated_pose_callback, this);
    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
}

void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr &msg){
    global_path = *msg;
    global_path_get_flag = true;
}

void LocalGoalCreator::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    estimated_pose = *msg;
    estimated_pose_get_flag = true;
    if(global_path_get_flag){
        local_goal_selector();
    }
}

/*Goal selector
Estimated_poseを受け取った後にその位置からある程度離れたところにあるゴールを選ぶ
Whileでやっているのは、ある程度距離が近いgoalを少しずつ離れたところに移動させる
これによってgoalとroombaが一定距離を保つように調整する。*/
void LocalGoalCreator::local_goal_selector(){
    double dist_x = estimated_pose.pose.position.x - global_path.poses[goal_index].pose.position.x;
    double dist_y = estimated_pose.pose.position.y - global_path.poses[goal_index].pose.position.y;
    double distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
    while(distance < local_goal_dist && goal_index+1 < int(global_path.poses.size())){
        distance += 0.1;
        goal_index += 1;
    }
    local_goal = global_path.poses[goal_index];
    local_goal.header.frame_id = "map";
    local_goal.header.stamp = ros::Time::now();
    local_goal.pose.orientation.w = 1;
}

void LocalGoalCreator::process(){
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(global_path_get_flag && estimated_pose_get_flag){
            pub_local_goal.publish(local_goal);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator local_goal_creator;
    local_goal_creator.process();
    return 0;
}
