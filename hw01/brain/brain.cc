///////////////////////
// Thomas Kaunzinger //
//                   //
// CS4610            //
// Homework 1        //
///////////////////////

// Imports
#include <iostream>
#include <math.h>
#include "robot.hh"

// Dumb macros for ez math because radians are smelly
#define d2r(degrees) ((degrees) * M_PI / 180.0)
#define r2d(radians) ((radians) * 180.0 / M_PI)

// Nat Tuck clogging up the namespace in the sample code smh
using std::cout;
using std::endl;

// Epic globals moment
const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

// Preliminary function declaration
bool turn_to(Robot*, double);

//////////////////////////
// ROBOT LOGIC CALLBACK //
//////////////////////////
void callback(Robot* robot) {
    double cur_x = robot->pos_x;
    double cur_y = robot->pos_y;
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;

    // Win condition
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    // Hit detection from starter code
    bool turn = false;
    for (LaserHit hit : robot->hits) {
        if (hit.range < 1.5) {
            if (hit.angle < 0.5 || hit.angle > (6.2 - 0.5)) {
                turn = true;
            }
        }
    }

    // Incredibly complex logic when you are about to hit a target
    // NOTE: This happens to work for this arena but isn't infallible
    if (turn) {
        cout << "OBStACLE: turning to 90 degrees" << endl;
        turn_to(robot, 90);	
    }
    
    // Base logic to gradually go towards the goal
    else {
        cout << "NO OBSTACLE: turning to ";
	    if (robot->pos_x < goal_x - 0.5) {
            cout << "0 degrees" << endl;
            turn_to(robot, 0);
    	}
        else if (robot->pos_x > goal_x + 0.5) {
            cout << "180 degrees" << endl;
            turn_to(robot, 180);
        }
        else if (robot->pos_y < goal_y - 0.5) {
            cout << "90 degrees" << endl;
            turn_to(robot, 90);
        }
        else if (robot->pos_y > goal_y + 0.5) {
            cout << "-90 degrees" << endl;
            turn_to(robot, -90);
        }

    }
}

// Makes the robot face a certain direction
// Returns true if already facing said direction within a 0.5 degree tolerance
bool turn_to(Robot* robot, double degrees) {
    bool done = false;
    double current = r2d(robot->pos_t);
    
    cout << "Cur: " << current << ", Target: " << degrees << ", Diff: ";

    double diff = degrees - current;
    diff = diff > 180 ? diff - 360 : diff;
    diff = diff < -180 ? diff + 360 : diff;

    cout << diff << ", Dir: ";

    if (-0.5 < diff && diff < 0.5) {
        robot->set_vel(5.0);
        robot->set_turn(d2r(0.0));
        cout << "0, Done: T" << endl;
        done = true;
    }
    else if (diff < 0) {
        robot->set_vel(2.0);
        robot->set_turn(d2r(45));
        cout << "-, Done: F" << endl;
    }
    else {
        robot->set_vel(2.0);
        robot->set_turn(d2r(-45));
        cout << "+, Done: F" << endl;
    }
    return done;
}

// Main function
int main(int argc, char* argv[]) {
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
