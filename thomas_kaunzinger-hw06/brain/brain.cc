///////////////////////
// Thomas Kaunzinger //
//                   //
// CS4610            //
// Homework 6        //
///////////////////////

// Imports
#include <iostream>
#include <thread>
#include <math.h>
#include <deque>
#include "robot.hh"
#include "viz.hh"

// Math macros
#define d2r(degrees) ((degrees) * M_PI / 180.0)
#define r2d(radians) ((radians) * 180.0 / M_PI)
#define clamp(min, x, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

// Clogged up namespace smh
using namespace std;

// Constants
#define k_wall          1.5
#define k_far           2.0
#define k_vel           1.5
#define k_vel_fast      3.0
#define k_mma_window    10      // Window size of the modified moving average position
#define k_minimum_mma   5
#define k_discard       3       // How many of the first MMA values to discard
                                // (because robo starts at 0,0 for some reason)

// States
#define FIRST_WALL  0x00
#define FIND_WALL   0x01
#define TURN_RIGHT  0x02
#define FOLLOW      0x03

// Global variables
int m_state = FIRST_WALL;
unordered_map<coord, cell_params> occupancy_grid;
float m_last_x = 0.0;
float m_last_y = 0.0;
deque<float> m_running_xs;
deque<float> m_running_ys;
bool m_purge_start = true;

// Function initializations
void find_wall(Robot* robot);
void turn_right(Robot* robot);
void follow(Robot* robot);

// Robot callback
// Credit to The Construct on YouTube for the simple wallfollow implementation
void callback(Robot* robot) {

    // float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = (clamp(0.0, robot->ranges[2].range, 2.1) + 
                 clamp(0.0, robot->ranges[3].range, 2.1) + 
                 clamp(0.0, robot->ranges[4].range, 2.1)) / 3.0;
    float frgt = (clamp(0.0, robot->ranges[1].range, 2.1) + 
                  clamp(0.0, robot->ranges[2].range, 2.1)) / 2.0;
    float flft = (clamp(0.0, robot->ranges[4].range, 2.1) + 
                  clamp(0.0, robot->ranges[5].range, 2.1)) / 2.0;

    if (m_state == FIRST_WALL && fwd > k_wall)                m_state = FIRST_WALL;
    else if (fwd > k_wall && flft > k_wall && frgt > k_wall)  m_state = FIND_WALL;
    else if (fwd < k_wall && frgt > k_wall && flft > k_wall)  m_state = TURN_RIGHT;
    else if (fwd > k_wall && frgt > k_wall && flft < k_wall)  m_state = FOLLOW;
    else if (fwd > k_wall && frgt < k_wall && flft > k_wall)  m_state = FIND_WALL;
    else if (fwd < k_wall && frgt > k_wall && flft < k_wall)  m_state = TURN_RIGHT;
    else if (fwd < k_wall && frgt < k_wall && flft > k_wall)  m_state = TURN_RIGHT;
    else if (fwd < k_wall && frgt < k_wall && flft < k_wall)  m_state = TURN_RIGHT;
    else if (fwd > k_wall && frgt < k_wall && flft < k_wall)  m_state = FIND_WALL;
    else                                                      m_state = FIND_WALL;

    
    // Determines what state to engage in
    switch(m_state) {
        case FIRST_WALL:
            robot->set_vel(k_vel, k_vel);
            break;
        case FIND_WALL:
            //cout << "FIND WALL - FLFT<" << flft << ">, FWD<"
            //    << fwd << ">, FRGT<" << frgt << ">" << endl;
            find_wall(robot);
            break;
        case TURN_RIGHT:
            //cout << "TURN RIGHT - FLFT<" << flft << ">, FWD<"
            //    << fwd << ">, FRGT<" << frgt << ">" << endl;
            turn_right(robot);
            break;
        case FOLLOW:
            //cout << "FOLLOW - FLFT<" << flft << ">, FWD<"
            //    << fwd << ">, FRGT<" << frgt << ">" << endl;
            follow(robot);
            break;
        default:
            //cout << "DEFAULT" << endl;
            robot->set_vel(k_vel, k_vel);
    }

    cout << "x<" << robot->pos_x << ">, y<" << robot->pos_y
        << ">, t<" << r2d(robot->pos_t) << ">" << endl;


    // Performs a modified moving average algorithm on the x and y positions
    float x_send, y_send;

    m_running_xs.push_front(robot->pos_x);
    m_running_ys.push_front(robot->pos_y);
    if (m_running_xs.size() > k_mma_window) {
        m_running_xs.pop_back();
    }
    if (m_running_ys.size() > k_mma_window) {
        m_running_ys.pop_back();
    }

    float running = 0.0;
    for (int i = 0; i < m_running_xs.size(); i++) {
        running += m_running_xs[i];
    }
    x_send = running / m_running_xs.size();

    running = 0.0;
    for (int i = 0; i < m_running_ys.size(); i++) {
        running += m_running_ys[i];
    }
    y_send = running / m_running_ys.size();


    // Purges some of the initial values because they are really badly initialized.
    // For some reason the program always starts at (0, 0), which destroys the
    // initial average.
    if (m_purge_start && m_running_xs.size() >= k_discard) {
        while (m_running_xs.size() > 0) {
            m_running_xs.pop_back();
        }
        while (m_running_ys.size() > 0) {
            m_running_ys.pop_back();
        }
        m_purge_start = false;
    }


    // Updates the occupancy grid once there's enough data
    if (m_running_xs.size() > k_minimum_mma) {
        for (auto hit : robot->ranges) {
            bool is_hit = false;
            if (hit.range < 100) {
                is_hit = true;
            }
            viz_hit(occupancy_grid,
                    x_send, y_send, robot->pos_t,
                    m_last_x, m_last_y,
                    clamp(0, hit.range, 2.0), -hit.angle, is_hit);
               // cout<<"["<<i<<"]: "<< hit.range<<"@"<<r2d(hit.angle)<<endl;
        }
    }
    m_last_x = x_send;
    m_last_y = y_send;
}

// Inches forward to a wall
void find_wall(Robot* robot) {
    robot->set_vel(-k_vel, k_vel_fast); 
}

// Turns the robot left
void turn_right(Robot* robot) {
   robot->set_vel(k_vel, -k_vel); 
}

// Follows straightish
void follow(Robot* robot) {
    robot->set_vel(k_vel, k_vel); 
}


// Executes the robot thread
void robot_thread(Robot* robot) {
    robot->do_stuff();
}

int main(int argc, char* argv[]) {
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    //robot->do_stuff();
    thread rthr(robot_thread, &robot);

    viz_run();

    return 0;
    //return viz_run();
}
