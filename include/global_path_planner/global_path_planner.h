#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

struct open
{
    int f;
    int g;
    int pre_x;
    int pre_y;
};

struct twod
{
    int x;
    int y;
};

struct ftwod
{
    float x;
    float y;
};

class AStarPath
{
    public:
        AStarPath();                //コンストラクタ作成
        void process();
    
    private:
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void thick_wall();
        void set_goal();
        void make_heuristic(int);
        void A_star();

        int hz;

        int row;
        int col;
        std::vector<std::vector<int>> map_grid;
        std::vector<std::vector<int>> map_grid_copy;

        std::vector<std::vector<int>> heuristic;

        std::vector<twod> goal;

        std::vector<std::vector<open>> close_list;
        std::vector<std::vector<open>> open_list;

        float res;
        int count;

        int gx;
        int gy;
        int x2;
        int y2;
        int x;
        int y;
        int old_x;
        int old_y;

        std::vector<twod> delta;

        int g;
        int h;
        int f[4];
        int fmin;
        int kmin;

        ftwod origin;
        twod parent;
        twod child;

        bool map_check;
        bool wall_checker;
        bool resign;
        bool heu_check;
        bool startpoint;
        bool path_check;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber sub_map;
        ros::Publisher pub_map;
        ros::Publisher pub_path;
        ros::Publisher pub_goal;

        nav_msgs::OccupancyGrid the_map;         //house map
        nav_msgs::Path global_path;             //マップ全体でのパス
        nav_msgs::Path checkpoint_path;               //チェックポイントまでのパス
        geometry_msgs::PoseStamped goal_point;
};
#endif