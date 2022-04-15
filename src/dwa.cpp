#include<stdio.h>

DWA::DWA():private_nh("~"){
    private_nh.getParam("hz", hz);                                  //looprate
    private_nh.getParam("roomba_radius", roomba_radius);            //turn radius
    private_nh.getParam("max_speed", max_speed);                    //max speed of roomba
    private_nh.getParam("min_speed", min_speed);                    //minimum speed of roomba
    private_nh.getParam("max_yawrate", max_yawrate);                //turn speed of roomba
    private_nh.getParam("max_accel", max_accel);


}