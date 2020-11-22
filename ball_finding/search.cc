// (hopefully) vaguely useful implementation of search.hh interface
// NOTE: ONCE THE MODEL IS DONE, PLEASE CHECK TODOs

/////////////
// IMPORTS //
/////////////
#include "search.hh"


////////////
// MACROS //
////////////

// Clamps between 2 values
#define clamp(min, x, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))


////////////////////////////////////////////
// CONSTANTS (please validate with model) //
////////////////////////////////////////////

#define fov             1.3962634   // Field of View    TODO

#define ball_r          0xFF        // Yellow Ball!     TODO
#define ball_g          0xFF
#define ball_b          0x00
#define color_tolerance 0x08        // +/- color tolerance

#define ball_d          1           // Diameter of the ball (in meters) TODO

#define turn_speed      1.5         // Wheel speed for self-rotation


//////////////////////
// ADDITIONAL STUBS //
//////////////////////

int _facing(Robot* robot, float target_angle, float tolerance);
bool _approx_equal(float a, float b, float tolerance);
float _turn_vel(Robot* robot, float total_turn_angle, float target_angle);
float _diff(float deg1, float deg2);

cv::Mat _make_ball_mask(cv::Mat pic);
int _get_column_of_ball(cv::Mat pic);


////////////////////
// IMPLEMENTATION //
////////////////////

// Finds the direction of the ball relative to the current angle
float get_direction_of_ball(cv::Mat pic) {

    // Gets a mask of the image filtering the ball
    cv::Mat ball_mask = _make_ball_mask(pic); 
    
    // Finds the location of the topmost pixel of the ball (which should also be the center)
    int column = _get_column_of_ball(ball_mask);

    // Finds the angle of this column
    int pic_width = pic.cols;
    float proportion = static_cast<float>(column - (pic_width / 2.0))
                        / static_cast<float>(pic_width / 2.0);
    float direction = proportion * (fov / 2.0);
    return direction;
}

// Finds the distance a ball is away given the current picture
float get_distance_from_ball(cv::Mat pic) {
    
    // Gets a mask of the image filtering the ball
    cv::Mat ball_mask = _make_ball_mask(pic); 
    
    // Finds the location of the topmost pixel of the ball (which should also be the center)
    int column = _get_column_of_ball(ball_mask);

    // TODO: Check column for height of pixels of ball

    // TODO: With height of ball in pixels, find visual angle

    // TODO: With visual angle of ball, extrapolate distance with size of ball

}

// Turns the robot and returns true if the ball is in the camera frame
// TODO: PLEASE CHECK THE ROBOT MODEL
bool search_for_ball(Robot* robot) {
    cv::Mat pic = robot->frame;
    float angle = get_direction_of_ball(pic, robot->pos_t);
    
    // If angle is 666 instead of a valid angle, start turning
    if (angle > (M_PI / 2.0)) {
        robot->set_vel(-turn_speed, turn_speed);
        return false;
    }

    // The ball has been located!
    robot->set_vel(0, 0);
    return true;
}

// Aligns to the ball when it's in the camera frame
// TODO: PLEASE CHECK THE ROBOT MODEL
bool turn_to_ball(Robot* robot) {
    cv::Mat pic = robot->frame;
    float angle = get_direction_of_ball(pic, robot->pos_t);

    // You are facing the ball!
    if (_approx_equal(angle, 0, 0.1)) {
        robot->set_vel(0, 0);
        return true;
    }

    // Sort of smooths it so you don't massively overshoot
    spd = clamp(1.0, 1.0 + 3.0 * angle, 2.0);

    // Further turning towards the ball!
    (angle < 0) ? robot->set_vel(spd, -spd) : robot->set_vel(-spd, spd);
    return false;
}


// Ended up being less useful than expected
//
// // Checks if the robot is facing a desired angle, and if not what direction it is in
// //
// //  1   ==  too far left
// //  0   ==  facing
// // -1   ==  too far right
// //
// // TODO: PLEASE CHECK ROBOT MODEL
// int _facing(Robot* robot, float target_angle, float tolerance) {
//     float diff = _diff(robot->pos_t, target_angle);
//     if (abs(diff) < tolerance) {
//         return 0;
//     }
//     int ret = (diff < 0) ? -1 : 1;
//     return ret;
// }

// Returns true if two floats are approximately equal within a tolerance
bool _approx_equal(float a, float b, float tolerance) {
    return abs(a - b) < tolerance;
}

// Nevermind this function doesn't seem very useful anymore
//
// // Gets the ideal current velocity of a turn at a point in during said turn
// // TODO: PLEASE CHECK ROBOT MODEL
// float _turn_vel(Robot* robot, float total_turn_angle, float target_angle) {
//     return clamp(1.0,
//                 1.0 + 3.0 * abs(diff(robot->pos_t, target_angle)) / total_turn_angle,
//                 2.0);
// }

// Returns the difference between two angles, accounting for complete circles
float _diff(float ang_1, float ang_2) {
    float d = ang_1 - ang_2;
    d = d >  (M_PI / 2) ? d - M_PI : d;
    d = d < -(M_PI / 2) ? d + M_PI : d;
    return d;
}

// Makes a mask of the picture that picks out the ball specifically
cv::Mat _make_ball_mask(cv::Mat pic) {
    namespace cv {
        Mat frame = pic;

        // NOTE: DEFAULTS TO BGR, NOT RGB
        Scalar low = Scalar(clamp(0, ball_b - color_tolerance, 255),
                            clamp(0, ball_g - color_tolerance, 255),
                            clamp(0, ball_r - color_tolerance, 255));
        Scalar high = Scalar(clamp(0, ball_b + color_tolerance, 255),
                            clamp(0, ball_g + color_tolerance, 255),
                            clamp(0, ball_r + color_tolerance, 255));

        // Creates a black and white mask of the image filtered by the ball color in
        // accordance to the above scalar tolerances
        Mat mask;
        inRange(frame, low, high, mask);

        return mask;
    }
}

// TODO!!!!!!!!!!
int _get_column_of_mask(cv::Mat pic) {
    return 0;
}

