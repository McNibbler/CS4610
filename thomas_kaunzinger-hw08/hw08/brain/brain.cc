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
#include <deque>
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
#define mma_window      10

// This is the color of the horribly grey sky
#define sky_r           0xb2
#define sky_g           0xb2
#define sky_b           0xb2

// Globals
deque<float> pix_scalar_deque;
mutex mx;


typedef lock_guard<mutex> guard;


// Returns the number of pixels high a wall is in a desired column of an image
int wall_dist_from_top(int column, cv::Mat pic) {
    //cam_show(pic);

    for (int row = 1; row < pic_height / scale_factor; row++) {
        //cout << column << ", " << row << endl;
        Vec3b color = pic.at<Vec3b>(cv::Point(column, row));
        
        //cout << +color.val[0] << ", " << +color.val[1] << ", " << +color.val[2] << endl;

        // When you find a color that is no longer that fugly grey, you hit the wall
        if (abs(color.val[0] - sky_r) > 5
            && abs(color.val[1] - sky_g) > 5
            && abs(color.val[2] - sky_b) > 5) {
                return row;
        }
    }

    // This shouldn't happen unless there is no wall (or floor?) detected at all
    return -1;
}

// Returns the visual angle given the wall's distance from the top of the screen in pixels
float get_visual_angle(int pixels_from_top) {
    return fov * static_cast<float>(100 - (2 * pixels_from_top)) / 100.0;
}


// Returns known constant S in the visual angle formula
// V = 2 arctan(S / 2D)
// Use when visual angle is known, but distance is not
//
// dist = get_pix_scalar() / (2 * tan(visual_angle / 2.0))
float get_pix_scalar() {
    //guard _g(mx);
    float running = 0;
    for (float s: pix_scalar_deque) {
        running += s;
    }
    return running / pix_scalar_deque.size();
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

    //cv::Mat pic = robot->frame;
    cv::Mat pic;
    cv::resize(robot->frame, pic, cv::Size(pic_width / scale_factor, pic_height / scale_factor));


    // Calibrates roughly the pixels per meter of distance within the center of the screen
    float dist = robot->range;

    // Calibrate based on distance to a wall.
    if (dist < 2.5) {

        int column = (pic_width / scale_factor) / 2;         
        int pixels = wall_dist_from_top(column, pic);

        // Tooooo close, man
        if (pixels <= 1) {
            robot->set_vel(-1.0, -1.0);
            return;
        }

        // Updates the MMA vector for pix_scalar
        float visual_angle = get_visual_angle(pixels);
        float pix_scalar = (tan(visual_angle / 2.0)) * (2.0 * dist);

        cout << endl << pixels << ", " << visual_angle * 180.0 / M_PI << endl;
        pix_scalar_deque.push_front(pix_scalar);

        if (pix_scalar_deque.size() > mma_window) {
            pix_scalar_deque.pop_back();
        }
    }
    //// If can't calibrate, just go towards a wall
    //else {
    //    robot->set_vel(1.0, 1.0);
    //    return;
    //}

    Pose pose(robot->pos_x, robot->pos_y, robot->pos_t);

    // Takes column samples from throughout the FOV
    // Determines the hits 
    for (int column = 0; column < pic_samps; column++) {
        
        float scangle = (static_cast<float>(column + 1) * (fov / static_cast<float>(pic_samps + 1)))
                            - (fov / 2);
        int pixels = wall_dist_from_top(column * (pic_width / scale_factor), pic);

        // Can't take a meaningful distance when it's this close
        if (pixels <= 1) {
            continue;
        }

        // Compensates for the infinite focal length of the camera
        float visual_angle = get_visual_angle(pixels);
        //float dist = (static_cast<float>(pixels) / get_pix_scalar()) / cos(scangle);
        float dist = get_pix_scalar() / (2 * tan(visual_angle / 2.0)) / cos(scangle);

        // Uses a similar function to the previous grid_apply_hit to update the occupancy grid
        // with the new location of this camera hit
        grid_apply_cam_hit(scangle, dist, pose);
    }
    grid_find_path(pose.x, pose.y, 20.0f, 0.0f);
    Mat view = grid_view(pose);
    viz_show(view);
    //cam_show(robot->frame);

    float ang = grid_goal_angle(pose);
    float trn = clamp(-1.0f, ang, 1.0f);
    float fwd = clamp(0.0f, robot->range, 2.0f);
    if (abs(ang) > 0.5) {
        robot->set_vel(-trn, + trn);
        return;
    }

    if (fwd > 1.0) {
        robot->set_vel(2.0f - trn, 2.0f + trn);
        return;
    }

    robot->set_vel(0.5f, -0.5f);
    usleep(1000);
}

void robot_thread(Robot* robot) {
    robot->do_stuff();
}


int
main(int argc, char* argv[])
{
    //cam_init();

    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    //robot.do_stuff();
    std::thread rthr(robot_thread, &robot);

    return viz_run(argc, argv);
}











