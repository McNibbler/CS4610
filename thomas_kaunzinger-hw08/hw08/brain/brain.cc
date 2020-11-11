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
#include <opencv2/imgproc.hpp>

#include "robot.hh"
#include "grid.hh"
#include "viz.hh"
#include "cam.hh"


using namespace std;


// Constants
#define fov             1.3962634
#define pic_samps       5
#define pic_width       800
#define pic_height      800
#define scale_factor    8       // Int scalar to reduce resolution by

// This is the color of the horribly grey sky
#define sky_r           0xb2
#define sky_g           0xb2
#define sky_b           0xb2

// Globals
float pix_scalar = 0.0;



// Returns the number of pixels high a wall is in a desired column of an image
int wall_dist_from_top(int column, cv::Mat pic) {
    
    for (int row = 0; row < pic_height; row++) {
        Vec3b color = pic.at<Vec3b>(cv::Point(column, row));

        // When you find a color that is no longer that fugly grey, you hit the wall
        if (abs(color.val[0] - sky_r) > 5
            || abs(color.val[1] - sky_g) > 5
            || abs(color.val[2] - sky_b) > 5) {
                return row;
        }
    }

    // This shouldn't happen unless there is no wall (or floor?) detected at all
    return -1;
}






/*
To view the camera image in time, you could press CTRL-T in Gazebo
, choosing the Topic-"~/tankbot0/tankbot/camera_sensor/link/camera/image", 
then a Image Window will pop up, in order to view the Image in time.
*/

void
callback(Robot* robot)
{

    if (robot->frame.empty()) {
        return;
    }



    cv::Mat pic = robot->frame;

    //cv::resize(robot->frame, pic, cv::Size(pic_width / scale_factor, pic_height / scale_factor));


    // Calibrates roughly the pixels per meter of distance within the center of the screen
    if (abs(pix_scalar) < 0.01) {

        float dist = robot->range;

        // Calibrate based on distance to a wall.
        if (dist < 2.0) {

            int column = pic_width / 2;          
            int pixels = wall_dist_from_top(column, pic);
            pix_scalar = static_cast<float>(pixels) / dist;

        }
        // If can't calibrate, just go towards a wall
        else {
            robot->set_vel(1.0, 1.0);
            return;
        }
    }

    Pose pose(robot->pos_x, robot->pos_y, robot->pos_t);

    // Takes column samples from throughout the FOV
    // Determines the hits 
    for (int column = 0; column < pic_samps; column++) {
        
        float scangle = static_cast<float>(column) * (fov / static_cast<float>(pic_samps));
        int pixels = wall_dist_from_top(column, pic);

        // Compensates for the infinite focal length of the camera
        float dist = (pixels * pix_scalar) / cos(scangle - (scangle / 2));

        // Uses a similar function to the previous grid_apply_hit to update the occupancy grid
        // with the new location of this camera hit
        grid_apply_cam_hit(scangle, dist, pose);
    }

    grid_find_path(pose.x, pose.y, 20.0f, 0.0f);
    Mat view = grid_view(pose);
    viz_show(view);
    //cam_show(robot->frame);

    float ang = grid_goal_angle(pose);
    float trn = clamp(-1.0f, 3 * ang, 1.0f);
    float fwd = clamp(0.0f, robot->range, 2.0f);
    if (abs(ang) > 0.5) {
        robot->set_vel(-trn, + trn);
        return;
    }

    if (fwd > 1.0) {
        robot->set_vel(2.0f - trn, 2.0f + trn);
        return;
    }

    robot->set_vel(0.0, 0.0);
    usleep(1000);
}

void robot_thread(Robot* robot) {
    robot->do_stuff();
}


int
main(int argc, char* argv[])
{
    cout << "here1" <<endl;
    cam_init();

    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    //robot.do_stuff();
    std::thread rthr(robot_thread, &robot);

    return viz_run(argc, argv);
}











