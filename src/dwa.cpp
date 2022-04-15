#include<dwa/dwa.h>
//constructors
DWA::DWA():private_nh("~"){
    private_nh.getParam("hz", hz);                                  //looprate
    private_nh.getParam("roomba_radius", roomba_radius);            //turn radius
    private_nh.getParam("max_speed", max_speed);                    //max speed of roomba
    private_nh.getParam("min_speed", min_speed);                    //minimum speed of roomba
    private_nh.getParam("max_yawrate", max_yawrate);                //turn speed of roomba
    private_nh.getParam("max_accel", max_accel);
    private_nh.getParam("max_dyawrate", max_dyawrate);
    private_nh.getParam("speed_reso", speed_reso);
    private_nh.getParam("yawrate_reso", yawrate_reso);              //resolution of yawrate
    private_nh.getParam("dt", dt);
    private_nh.getParam("predict_time", predict_time);
    private_nh.getParam("heading_score_gain", heading_score_gain);
    private_nh.getParam("velocity_score_gain", velocity_score_gain);
    private_nh.getParam("dist_score_gain", dist_score_gain);
    private_nh.getParam("goal_tolerance", goal_tolerance);

    //subscriber
    local_goal_sub = nh.subscribe("/local_goal", 1, &DWA::local_goal_callback, this);
    obstacle_poses_sub = nh.subscribe("/obstacle_poses", 1, &DWA::obstacle_poses_callback, this);

    //publisher
    roomba_control_pub = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    local_paths_pub = nh.advertise<nav_msgs::Path>("local_paths", 1);
    best_local_path_pub = nh.advertise<nav_msgs::Path>("best_local_path", 1);
    local_goal_point_pub = nh.advertise<geometry_msgs::PointStamped>("local_goal_point", 1);

    local_goal_point.header.frame_id = "base_link";
    previous_input = {0.0, 0.0};
}