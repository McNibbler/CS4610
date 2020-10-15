///////////////////////
// Thomas Kaunzinger //
//                   //
// CS4610            //
// Homework 5        //
///////////////////////

// God I'm so glad to be using a real programming language again

// Uncomment to wall crawl backwards
//#define INVERSE

// Imports
#include <iostream>
#include <math.h>
#include <queue>
#include "robot.hh"

// Silly macros to make math easier
#define d2r(degrees) ((degrees) * M_PI / 180.0)
#define r2d(radians) ((radians) * 180.0 / M_PI)
#define clamp(min, x, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

// Clogged up namespace smh I'm sorry
using namespace std;

// Regular constants
#define k_num_rows      12
#define k_goal_row      10
#define k_row_length    ((3.0 * 13) - 2.0)  // length of row 0 and row k_num_rows - 1 in meters
#define k_hit_distance  0.5
#define k_backup        0.7
#define k_max_range     2.0
#define k_fwd_dist      1.6     // The distance to climb forward after reaching the end of a wall

// Direction constants
#ifdef INVERSE
    #define SOUTH       90
    #define NORTH       -90
#else
    #define SOUTH       -90
    #define NORTH       90
#endif
#define EAST            0
#define WEST            180

// State constants 
#define TURN_SOUTH      0x00
#define REACH_SOUTH     0x01
#define POST_SOUTH_BACK 0x02
#define TURN_NORTH      0x03
#define CALIBRATE       0x04
#define POST_CAL_BACK   0x05
#define TURN_WEST       0x06
#define REACH_NORTHWEST 0x07
#define POST_NW_BACK    0x08 
#define ORIENT_CRAWL    0x09
#define WALL_CRAWL      0x0A
#define WALL_CRAWL_TURN 0x0B
#define WALL_CRAWL_HIT  0x0C
#define WALL_CRAWL_GONE 0x0D
#define FINISH_ROW_TURN 0x0E
#define FINISH_ROW_FWD  0x0F
#define FINISH_ROW_CENT 0x10
#define FINISH_ROW_SCAN 0x11
#define FINISH_ROW_BACK 0x12
#define FINISH_ROW_CONT 0x13

// Running state
int m_state;

// Running variables
int m_scan_dir;
int m_current_row;
int m_ticks_per_meter;
int m_running_ticks;
int m_new_direction;
int m_forward_ticks;
bool m_continue_finish_row_scan;
bool m_ascending;
bool m_check_back;
float m_temp_dir;

// Instantiating functions
int facing(Robot* robot, float target_degrees, float tolerance);
int facing(Robot* robot, float target_degrees);
bool approx_equal(float a, float b, float tolerance);
float turn_vel(Robot* robot, float total_deg_of_turn, float target_deg);
float diff(float deg1, float deg2);


// Robot logic
// NOTE TO SELF: Interesting robot params...
// -- set_vel(vel_left, vel_right)
// -- pos_t
// -- range
void callback(Robot* robot) {

    // Instantiating the running velocity since it's not possible in a switch statement
    float vel = 4.5;            // Straight velocity
    float t_vel = 1.0;          // Turn velocity (use turn_vel() for more accurate velocity curves)
    float b_vel = -1.0;         // Velocity for backing up from a wall
    float a_vel = 0.5;          // Velocity delta for angle correction when going straight
    float c_vel = vel + a_vel;  // Correction velocity (used in conjunction with vel and a_vel)

    cout << endl;
    cout << "CURRENT ROW: " << m_current_row << endl;
    cout << "RANGE: " << robot->range << endl; // << ", PREV: " << m_last_range << endl;
    cout << "ANGLE: " << r2d(robot->pos_t) << endl;

    // Random variable instantiation
    int row_add = 0;

    // State machine
    switch(m_state) {

        // Initial state: turns to south wall
        case TURN_SOUTH:
            cout << "TURN SOUTH" << endl;
            if (facing(robot, SOUTH) == 0) {
                m_state = REACH_SOUTH;
                robot->set_vel(0, 0);
                break;
            }
            t_vel = turn_vel(robot, 90, SOUTH);
            (facing(robot, SOUTH) > 0) ?
                robot->set_vel(t_vel, -t_vel) : robot->set_vel(-t_vel, t_vel);
            break;

        // Reaches south wall
        case REACH_SOUTH:
            cout << "REACH SOUTH" << endl;
            if (facing(robot, SOUTH) == 0) {
                robot->set_vel(vel, vel);
            }
            else {
                (facing(robot, SOUTH) > 0) ?
                    robot->set_vel(c_vel, vel) : robot->set_vel(vel, c_vel);
            }

            if (robot->range < k_hit_distance) {
                m_state = POST_SOUTH_BACK;
            }
            break;

        // Backs up slightly when reaching the south wall
        case POST_SOUTH_BACK:
            cout << "POST SOUTH BACK" << endl;
            if (robot->range < k_backup) {
                robot->set_vel(b_vel, b_vel);
            }
            else {
                m_state = TURN_NORTH;
            }
            break;

        // Turns to north wall
        case TURN_NORTH:
            cout << "TURN NORTH" << endl;
            if (facing(robot, NORTH) == 0) {
                m_state = CALIBRATE;
                robot->set_vel(0, 0);
                m_running_ticks = 0;
                break;
            }
            t_vel = turn_vel(robot, 180, NORTH);
            (facing(robot, NORTH) > 0) ?
                robot->set_vel(t_vel, -t_vel) : robot->set_vel(-t_vel, t_vel);
            break;

        // Calibrates ticks per meter @vel based on time to get from south wall to north
        case CALIBRATE:
            cout << "CALIBRATE" << endl;

            if (facing(robot, NORTH) == 0) {
                robot->set_vel(vel, vel);
            }
            else {
                (facing(robot, NORTH) > 0) ?
                    robot->set_vel(c_vel, vel) : robot->set_vel(vel, c_vel);
            }

            m_running_ticks++;
            if (robot->range < k_hit_distance) {
                m_state = POST_CAL_BACK;
                m_ticks_per_meter = (int)floor(m_running_ticks / (k_row_length - (k_backup + k_hit_distance)));
            }
            break;

        // Backs up slightly after a calibration
        case POST_CAL_BACK:
            if (robot->range < k_backup) {
                robot->set_vel(b_vel, b_vel);
            }
            else {
                m_state = TURN_WEST;
            }
            break;

        // Turns towards the west wall
        case TURN_WEST:
            cout << "TURN WEST" << endl;
            if (facing(robot, WEST) == 0) {
                m_state = REACH_NORTHWEST;
                robot->set_vel(0, 0);
                break;
            }
            t_vel = turn_vel(robot, 90, WEST);
            (facing(robot, WEST) > 0) ?
                robot->set_vel(t_vel, -t_vel) : robot->set_vel(-t_vel, t_vel);
            break;

        // Gets the robot in the absolute most northwest corner of the maze
        case REACH_NORTHWEST:
            cout << "REACH NORTHWEST" << endl;

            if (facing(robot, WEST) == 0) {
                robot->set_vel(vel, vel);
            }
            else {
                (facing(robot, WEST) > 0) ?
                    robot->set_vel(c_vel, vel) : robot->set_vel(vel, c_vel);
            }

            if (robot->range < k_hit_distance) {
                m_state = POST_NW_BACK;
            }
            break;

        // Backs up slightly after reaching NW
        case POST_NW_BACK:
            cout << "POST NW BACK" << endl;
            if (robot->range < k_backup) {
                robot->set_vel(b_vel, b_vel);
            }
            else {
                m_state = ORIENT_CRAWL;
            }
            break;


        // Aligns the robot east, and begins the wall crawl
        case ORIENT_CRAWL:
            cout << "ORIENT CRAWL" << endl;
            if (facing(robot, EAST) == 0) {
                m_state = WALL_CRAWL;
                robot->set_vel(0, 0);
                m_running_ticks = 0;
                m_new_direction = EAST;
                break;
            }
            t_vel = turn_vel(robot, 180, EAST);
            (facing(robot, EAST) > 0) ?
                robot->set_vel(t_vel, -t_vel) : robot->set_vel(-t_vel, t_vel);
            break;

        // Executes the wall crawl
        case WALL_CRAWL:
            cout << "WALL CRAWL MAIN" << endl;
            // Executes the turns
            if (m_current_row == k_goal_row && !m_continue_finish_row_scan) {
                m_state = FINISH_ROW_CENT;
                m_check_back = true;
                break;
            }

            // Wall hit
            else if (robot->range < k_hit_distance) {
                m_state = WALL_CRAWL_HIT;
                break;
            }

            // Wall gone
            else if (robot->range > k_max_range) {
                m_state = WALL_CRAWL_GONE;
                m_forward_ticks = 0;
                break;
            }

            if (facing(robot, m_new_direction) == 0) {
                robot->set_vel(vel, vel);
            }
            else {
                (facing(robot, m_new_direction) > 0) ?
                    robot->set_vel(c_vel, vel) : robot->set_vel(vel, c_vel);
            }

            m_running_ticks++;

            break;
        
        // Wall crawl when you hit a wall
        case WALL_CRAWL_HIT:
            cout << "WALL CRAWL HIT" << endl;
            
            // Backs up slightly from wall
            if (robot->range < k_backup) {
                robot->set_vel(b_vel, b_vel);
                break;
            }

            // Adds or removes rows depending on direction, and then commands to turn
            if (facing(robot, EAST, 5) == 0) {
                row_add = (int) floor(m_running_ticks / (4 * m_ticks_per_meter - k_backup * 2));
                m_current_row += row_add;
                if (row_add > 0 && m_current_row == k_goal_row) {
                    m_continue_finish_row_scan = false;
                }
                m_ascending = true;
                m_new_direction = SOUTH;
            }
            else if (facing(robot, WEST, 5) == 0) {
                row_add = (int) floor(m_running_ticks / (4 * m_ticks_per_meter - k_backup * 2));
                m_current_row -= row_add; 
                if (row_add > 0 && m_current_row == k_goal_row) {
                    m_continue_finish_row_scan = false;
                }
                m_ascending = false;
                m_new_direction = NORTH;
            }
            else if (facing(robot, NORTH, 5) == 0) {
                m_new_direction = EAST;
            }
            else {
                m_new_direction = WEST;
            }

            m_state = WALL_CRAWL_TURN;
            break;

        // Wall crawl when the walls disappear
        case WALL_CRAWL_GONE:
            cout << "WALL CRAWL WALL GONE" << endl;
            
            // Crawls past the end of the wall
            if (m_forward_ticks < (m_ticks_per_meter * k_fwd_dist)) {
                robot->set_vel(vel, vel);
                m_forward_ticks++;
                break;
            }
                
            if (facing(robot, EAST, 5) == 0) {
                m_current_row += (int) floor((m_running_ticks + m_forward_ticks) / (4 * m_ticks_per_meter - k_backup) + 1);
                if (m_current_row == k_goal_row) {
                    m_continue_finish_row_scan = false;
                }
                m_ascending = true;
                m_new_direction = NORTH;
            }
            else if (facing(robot, WEST, 5) == 0) {
                m_current_row -= (int) floor((m_running_ticks + m_forward_ticks) / (4 * m_ticks_per_meter - k_backup) + 1);
                if (m_current_row == k_goal_row) {
                    m_continue_finish_row_scan = false;
                }
                m_ascending = false;
                m_new_direction = SOUTH;
            }
            else if (facing(robot, NORTH, 5) == 0) {
                m_new_direction = WEST;
            }
            else {
                m_new_direction = EAST;
            }

            m_state = WALL_CRAWL_TURN;
            break;

        // Turning case for mid wall crawl
        case WALL_CRAWL_TURN:
            cout << "WALL CRAWL TURN" << endl;
            if (facing(robot, m_new_direction) == 0) {
                m_state = WALL_CRAWL;
                robot->set_vel(0, 0);
                m_running_ticks = 0;
                break;
            }
            t_vel = turn_vel(robot, 90, m_new_direction);
            (facing(robot, m_new_direction) > 0) ?
                robot->set_vel(t_vel, -t_vel) : robot->set_vel(-t_vel, t_vel);
            break;

        // Centers the root in the finishing row
        case FINISH_ROW_CENT:
            cout << "FINISH ROW CENT" << endl;
            
            m_temp_dir = m_ascending ? EAST : WEST;
            if (facing(robot, m_temp_dir) == 0) {
                m_state = FINISH_ROW_FWD;
                m_forward_ticks = 0;
                break;
            }
            t_vel = turn_vel(robot, 90, m_temp_dir);
            (facing(robot, m_temp_dir) > 0) ?
                robot->set_vel(t_vel, -t_vel) : robot->set_vel(-t_vel, t_vel);
            break;

        case FINISH_ROW_FWD:
            cout << "FINISH ROW FWD" << endl;
    
            // moves a little bit forward
            if (m_forward_ticks < (m_ticks_per_meter * 0.5)) {
                robot->set_vel(vel, vel);
                m_forward_ticks++;
                m_running_ticks++;
            }
            else {
                m_state = FINISH_ROW_TURN;
            }
            break;
            

        // Turns towards the ends of the hallways
        case FINISH_ROW_TURN:
            cout << "FINISH ROW TURN" << endl;
            m_temp_dir = m_check_back ? -m_new_direction : m_new_direction;

            if (facing(robot, m_temp_dir) == 0) {
                m_state = FINISH_ROW_SCAN;
                break;
            }
            t_vel = turn_vel(robot, 180, m_temp_dir);
            (facing(robot, m_temp_dir) > 0) ?
                robot->set_vel(t_vel, -t_vel) : robot->set_vel(-t_vel, t_vel);
            break;

        // Scans the last row (this may or may not be the last step!)
        case FINISH_ROW_SCAN:
            cout << "FINISH ROW SCAN" << endl;
            
            if (robot->range < k_hit_distance) {
                m_state = FINISH_ROW_BACK;
                break;
            }

            if (facing(robot, m_temp_dir) == 0) {
                robot->set_vel(vel, vel);
            }
            else {
                (facing(robot, m_temp_dir) > 0) ?
                    robot->set_vel(c_vel, vel) : robot->set_vel(vel, c_vel);
            }

            break;

        // Backs up when a wall is hit and signals the robot to turn around
        case FINISH_ROW_BACK:
            cout << "FINISH ROW BACK" << endl;
            // Backs up slightly from wall
            if (robot->range < k_backup) {
                robot->set_vel(b_vel, b_vel);
            }
            else {
                m_state = m_check_back ? FINISH_ROW_TURN : FINISH_ROW_CONT;
                m_check_back = !m_check_back;
            }
            break;


        // Sucks to suck back to crawling for you
        case FINISH_ROW_CONT:
            cout << "FINISH ROW CONT" << endl;
            m_continue_finish_row_scan = true;
            m_state = WALL_CRAWL;
            break;

        // This should never happen
        default:
            cout << "DEFAULT" << endl;
            robot->set_vel(0.0, 0.0);
    }
    
}

// Helper functions
// 1 == is too far left, 0 == facing, -1 == too far right
int facing(Robot* robot, float target_degrees, float tolerance) {
    float diff = r2d(robot->pos_t) - target_degrees;
    diff = diff > 180 ? diff - 360 : diff;
    diff = diff < -180 ? diff + 360 : diff;

    if (abs(diff) < tolerance) {
        return 0;
    }
    int ret = (diff < 0) ? -1 : 1;
    return ret;
}

// Overload with a default tolerance
int facing(Robot* robot, float target_degrees) {
    return facing(robot, target_degrees, 0.01);
}

// Returns true if two floats are approximately equal within a tolerance
bool approx_equal(float a, float b, float tolerance) {
    return abs(a - b) < tolerance;
}

// Gets the ideal current velocity of a turn at a given point during said turn
float turn_vel(Robot* robot, float total_deg_of_turn, float target_deg) {
    return clamp(0.5, 0.5 + 4.0 * abs(diff(r2d(robot->pos_t), target_deg)) / total_deg_of_turn, 2.0);
}

// Returns the difference between two angles, accounting for negatives
float diff(float deg1, float deg2) {
   float d = deg1 - deg2;
   d = d > 180 ? d - 360 : d;
   d = d < -180 ? d + 360 : d;
   return d;
}


// Main
int main(int argc, char* argv[]) {
    m_current_row = 0;
    m_state = TURN_SOUTH;
    m_continue_finish_row_scan = false;
    m_ascending = true;
    m_temp_dir = 0;

    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
