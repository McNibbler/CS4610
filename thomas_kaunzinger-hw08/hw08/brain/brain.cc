// Thomas Kaunzinger
// Homework 8
//
// Cameras!
//
// Based on pathfinding solution for homework 7 provided by Nat Tuck



#include <iostream>
#include <thread>
#include <algorithm>
#include <math.h>

#include "robot.hh"
#include "grid.hh"
#include "viz.hh"
#include "cam.hh"

using namespace std;

/*
To view the camera image in time, you could press CTRL-T in Gazebo
, choosing the Topic-"~/tankbot0/tankbot/camera_sensor/link/camera/image", 
then a Image Window will pop up, in order to view the Image in time.
*/

void
callback(Robot* robot)
{


    cam_show(robot->frame);
    // TODO: Update occupancy grid with camera logic here 
 

    Pose pose(robot->pos_x, robot->pos_y, robot->pos_t);
    grid_find_path(pose.x, pose.y, 20.0f, 0.0f);
    Mat view = grid_view(pose);
    viz_show(view);

    float ang = grid_goal_angle(pose);
    float trn = clamp(-1.0, 3 * ang, 1.0);



    cout << "here2" <<endl;

    robot->set_vel(0.0, 0.0);
}

int
main(int argc, char* argv[])
{
    cout << "here1" <<endl;
    cam_init();

    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}











