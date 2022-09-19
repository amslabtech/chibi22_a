#include "global_path_planner/global_path_planner.h"

AStarPath::AStarPath():private_nh("~")
{
    private_nh.param("hz",hz,{10});                                     //実行した後に、hzの値を変えることができる。　{}はデフォルト値
    private_nh.param("map_check",map_check,{false});
    private_nh.param("wall_check",wall_check,{false});
    private_nh.param("heu_check",heu_check,{false});
    private_nh.param("path_check",path_check,{false});
    sub_map = nh.subscribe("/map",10,&AStarPath::map_callback,this);    //"/map"からマップをもらい、callback関数に送る
    pub_path = nh.advertise<nav_msgs::Path>("/path",1);
}

void AStarPath::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
//const nav_msgs::Odometry::ConstPtr は、const型(内容を書き換えられない)、nav_msgsパッケージに含まれる、Odometry型のメッセージの、const型ポインタを表している
//&msgの&は、参照型(内容を書き換えられるように変数を渡すことができる)という意味ですが、(const型なので)ここでは特に気にする必要はない
{
    if(map_check){
        return;     //exit from processing on the way
    }
    else
    {
        the_map = *msg;
        row = the_map.info.height;          //row = 4000
        col = the_map.info.width;           //col = 4000
        map_grid = std::vector< std::vector<int> >(row,std::vector<int>(col,0));
        map_grid_copy = std::vector<std::vector<int>>(row,std::vector<int>(col,0));

        //change 1D the_map to 2D
        for(int i=0; i<row; i++)
        {
            for(int j=0; j<col; j++)
            {
                map_grid[i][j] = the_map.data[i+j*row];
            }
        }
        // origin mean point which is edge of left down
        origin.x = the_map.info.origin.position.x;      //origin.x = -100
        origin.y = the_map.info.origin.position.y;      //origin.y = -100
        map_check = true;
    }
}

void AStarPath::thick_wall()
{
    if(!wall_check)
    {
        for(int i=0; i<row; i++)
        {
            for(int j=0; j<col; j++)
            {
                map_grid_copy[i][j] = map_grid[i][j];
            }
        }

        //add wall
        for(int i=5; i<row-5; i++)
        {
            for(int j=5; j<col; j++)
            {
                if(map_grid_copy[i][j]==100)
                {
                    map_grid[i+1][j]=100;       //up+1
                    map_grid[i+1][j+1]=100;     //up+1, left+1
                    map_grid[i+1][j+2]=100;     //up+1, left+2
                    map_grid[i][j+1]=100;       //left+1
                    map_grid[i][j+2]=100;       //left+2
                    map_grid[i+2][j]=100;       //up+2
                    map_grid[i+2][j+1]=100;     //up+2, left+1
                    map_grid[i+2][j+2]=100;     //up+2, left+2
                    map_grid[i][j-1]=100;       //right+1
                    map_grid[i][j-2]=100;       //right+2
                    map_grid[i-1][j]=100;       //down+1
                    map_grid[i-1][j-1]=100;     //down+1, right+1
                    map_grid[i-1][j-2]=100;     //down+1, right+2
                    map_grid[i-2][j]=100;       //down+2
                    map_grid[i-2][j-1]=100;     //down+2, right+1
                    map_grid[i-2][j-2]=100;     //down+2, right+2
                }
            }
        }

        //resolution is 0.05m
        resolution = the_map.info.resolution;

        //edit the_map.data (the_map.data mean 1D data , map_grid mean 2D data)
        for(int i=0; i<row; i++)
        {
            for(int j=0; j<col; j++)
            {
                the_map.data[i+j*row] = map_grid[i][j];
            }
        }
        wall_check = true;
    }
}

void AStarPath::set_goal()
{
    goal = {{2000,2000},{2320,2010},{2320,2290},{2000,2290},{1660,2280},{1660,2000},{2000,2000}};
}

//heuristic process that caluculate path to check point
void AStarPath::make_heuristic(int next)        //next >= 1
{
    heuristic = std::vector<std::vector<double>>(row,std::vector<double>(col,0));
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<col; j++)
        {
            heuristic[i][j] = sqrt((goal[next].x - i)*(goal[next].x - i) + (goal[next].y - j)*(goal[next].y - j));
        }
    }
}

void AStarPath::A_star()
{
    //std::vector<twod> delta;
    delta = {{0,-1},{-1,0},{0,1},{1,0},{1,1},{1,-1},{-1,1},{-1,-1}};
    //go left {0,-1} = delta[0]
    //go up {-1,0} = delta[1]
    //go right {0,1} = delta[2]
    //go down {1,0} = delta[3]
    //go down right {1,1} = delta[4]
    //go down left = {1,-1} = delta[5]
    //go up right = {-1,1} = delta[6]
    //go up left = {-1,-1} = delta[7]

    open_list = std::vector<std::vector<open>>(row,std::vector<open>(col));

    //A*process that caluculate path to check point
    for(int i=0; i<goal.size()-1; i++)
    {
        checkpoint_path.poses.clear();    //clear() : clear elements //"member function" of vector
        make_heuristic(i+1);        //make heuristic // next is over 1 , so i+1
        resign = false;            //judgement of goal is miss
        goal_point.header.frame_id = "map";         //rviz indicate this frame_id
        goal_point.pose.position.x = goal[i+1].x;   //local goal of x
        goal_point.pose.position.y = goal[i+1].y;   //local goal of y
        //std::cout<<"goal_point.pose.position.x  :  "<<goal_point.pose.position.x<<std::endl;
        //std::cout<<"goal_point.pose.position.y  :  "<<goal_point.pose.position.y<<std::endl;
        close_list = std::vector<std::vector<open>>(row,std::vector<open>(col));

        //initializetion of list
        for(int r=0; r<row; r++)
        {
            for(int c=0; c<col; c++)
            {
                close_list[r][c].f = 0;
                close_list[r][c].g = 0;
                close_list[r][c].pre_x = 0;
                close_list[r][c].pre_y = 0;
                open_list[r][c].f = 0;
                open_list[r][c].g = 0;
                open_list[r][c].pre_x = 0;
                open_list[r][c].pre_y = 0;
            }
        }

        gx = goal[i+1].x;   //gx is goal point
        gy = goal[i+1].y;   //gy is goal point
        x = goal[i].x;      //x is now point ( grid )
        y = goal[i].y;      //y is now point ( grid )
        g = 0;              //cost function

        while(!resign)              //if !resign is ture // resign is false
        {
            if(x == gx && y == gy)  //if now point is goal point
            {
                resign = true;      //search finish
            }
            else                    //if now point is not goal point
            {
                g += 1;             //add cost to g
                // following "for" sentence is 4 direction check
                for(int j=0; j<delta.size(); j++)
                {
                    //order is "left -> up -> right -> down"
                    x2 = x + delta[j].x;                            //dicide 4 direction
                    y2 = y + delta[j].y;                            //dicide 4 direction
                    if(x2>=0 && x2<row && y2>=0 && y2<col)          //row and col check mean x2 and y2 are in grid or out
                    {
                        //if x2 and y2 are in grid
                        if(close_list[x2][y2].g == 0 && map_grid[x2][y2] != 100)
                        {
                            // if close_list is No check (close_list is 0) and map_grid is true
                            // (if close_list has already checked , the if sentence is not used)
                            open_list[x2][y2].g = g;
                            open_list[x2][y2].f = f[j];
                            open_list[x2][y2].pre_x = x;
                            open_list[x2][y2].pre_y = y;
                            f[j] = g + heuristic[x2][y2];           //calicurate cost of f(n)
                            close_list[x2][y2].pre_x = x;           //substitution now_x to pre_x
                            close_list[x2][y2].pre_y = y;           //substitution now_y to pre_y
                            close_list[x2][y2].g = 1;               //substitution
                        }
                        else
                        {
                            //if close_list has already checked
                            f[j] = 1000;
                        }
                    }
                }
                //initializetion
                f_min = f[0];
                k_min = 0;
                //search minimum f cost
                for(int k=0; k<delta.size(); k++)
                {
                    if(f[k]<f_min)
                    {
                        f_min = f[k];
                        k_min = k;
                    }
                }
                //now point update
                if(f_min!=1000)
                {
                    old_x = x;                  //substitution now point (x) to old_x
                    old_y = y;                  //substitution now point (y) to old_y
                    x = x + delta[k_min].x;      //update now point (x) // plus most smallest cost delta[k_min].x
                    y = y + delta[k_min].y;      //update now point (y) // plus most smallest cost delta[k_min].y
                }
            }
        }

        //std::cout<<"bad"<<std::endl;

        startpoint = false;
        checkpoint_path.header.frame_id = "map";

        geometry_msgs::PoseStamped point;

        //"(now_point-center_grid)*0.05" indicate length (not grid number)
        //so point.pose.position is length
        point.pose.position.x = (x-row/2)*resolution;      //"row/2" is center of grid number (2000)
        point.pose.position.y = (y-col/2)*resolution;      //"col/2" is center of grid number (2000)

        checkpoint_path.poses.push_back(point);     //push_back add element to end of array
        //x, y indicate goal point (grid)
        parent.x = close_list[x][y].pre_x;
        parent.y = close_list[x][y].pre_y;

        while(!startpoint)
        {
            //goal[i] indicate grid
            if(parent.x == goal[i].x && parent.y == goal[i].y)
            //whether parent is equl to real goal point or not
            {
                point.pose.position.x = (parent.x-row/2)*resolution;
                point.pose.position.y = (parent.y-col/2)*resolution;
                checkpoint_path.poses.push_back(point);
                startpoint = true;
            }
            else
            {
                point.pose.position.x = (parent.x-row/2)*resolution;
                point.pose.position.y = (parent.y-col/2)*resolution;
                checkpoint_path.poses.push_back(point);
                //child.x = parent.x;
                //child.y = parent.y;
                parent.x = close_list[parent.x][parent.y].pre_x;
                parent.y = close_list[parent.x][parent.y].pre_y;
            }
        }
        std::reverse(checkpoint_path.poses.begin(),checkpoint_path.poses.end());
        global_path.header.frame_id = "map";            //rviz indicate this frame_id
        global_path.poses.insert(global_path.poses.end(), checkpoint_path.poses.begin(), checkpoint_path.poses.end());
        std::cout<<"i="<<i+1<<" path :"<< checkpoint_path.poses.size()<<std::endl;
    }
}

void AStarPath::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(map_check && !path_check)    //map_checker and path_check are bool
        {
            thick_wall();                   //wall add
            set_goal();                     //decide goll
            A_star();                       //A*process
            map_check = false;
            path_check = true;
        }

        if(path_check==true)
        {
            pub_path.publish(global_path);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planner");           //node name "Global_path_planner"
    AStarPath astarpath;
    astarpath.process();
    return 0;
}


