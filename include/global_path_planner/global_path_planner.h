#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

struct open
{
    double f;
    int g;
    int pre_x;
    int pre_y;
};

struct twod
{
    double x;
    double y;
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

        std::vector<std::vector<double>> heuristic;

        std::vector<twod> goal;

        std::vector<std::vector<open>> close_list;
        std::vector<std::vector<open>> open_list;

        float resolution;

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
        double f[8];
        double f_min;
        double k_min;

        ftwod origin;
        twod parent;
        twod child;

        bool map_check;
        bool wall_check;
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