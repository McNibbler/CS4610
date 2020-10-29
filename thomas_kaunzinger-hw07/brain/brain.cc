///////////////////////
// Thomas Kaunzinger //
//                   //
// CS4610            //
// Homework 6        //
///////////////////////

// Imports
#include <iostream>
#include <thread>
#include <mutex>
#include <math.h>
#include <deque>
#include <vector>
//#include <X11/Xlib.h>
#include "robot.hh"
#include "viz.hh"

// Math macros
#define d2r(degrees) ((degrees) * M_PI / 180.0)
#define r2d(radians) ((radians) * 180.0 / M_PI)
#define clamp(min, x, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

// Clogged up namespace smh
using namespace std;

// Constants
#define k_goal_x                20.0    // Destination location
#define k_goal_y                0.0
#define k_end_tolerance         0.5

#define k_wall                  1.5
#define k_far                   2.0
#define k_vel                   1.5
#define k_vel_fast              3.0
#define k_mma_window            10      // Window size of the modified moving average position
#define k_minimum_mma           5
#define k_discard               3       // How many of the first MMA values to discard
                                        // (because robo starts at 0,0 for some reason)
#define k_wall_confidence       0.75    // Scalar coefficient on confidence that a cell is a wall
#define k_hit_priority          3       // Copied from viz.cc cus im lazy


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
vector<coord> m_path;

mutex mut;

// Function initializations
void find_wall(Robot* robot);
void turn_right(Robot* robot);
void follow(Robot* robot);
bool is_wall(cell_params cp);
node* make_node(coord c);
coord make_coord(int x, int y);
vector<coord> neighbors(coord c);
void astar_thread();
bool astar(float current_x, float current_y, float destination_x, float destination_y); 
float get_current_mma_x(); 
float get_current_mma_y(); 


/////////////////////////
// ROBOT FUNCTIONALITY //
/////////////////////////

// Robot callback
// Credit to The Construct on YouTube for the simple wallfollow implementation
void callback(Robot* robot) {

    // float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = (clamp(0.0, robot->ranges[2].range, 2.05) + 
                 clamp(0.0, robot->ranges[3].range, 2.05) + 
                 clamp(0.0, robot->ranges[4].range, 2.05)) / 3.0;
    float frgt = (clamp(0.0, robot->ranges[1].range, 2.05) + 
                  clamp(0.0, robot->ranges[2].range, 2.05)) / 2.0;
    float flft = (clamp(0.0, robot->ranges[4].range, 2.05) + 
                  clamp(0.0, robot->ranges[5].range, 2.05)) / 2.0;

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


    // Updates the running x and y positions for the MMA algorithm
    mut.lock();

    m_running_xs.push_front(robot->pos_x);
    m_running_ys.push_front(robot->pos_y);
    if (m_running_xs.size() > k_mma_window) {
        m_running_xs.pop_back();
    }
    if (m_running_ys.size() > k_mma_window) {
        m_running_ys.pop_back();
    }

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

    // Also copies the path to send to viz_hit to draw the path
    vector<coord> path_to_send = m_path;
    mut.unlock();


    // Gets the MMA
    float x_send = get_current_mma_x();
    float y_send = get_current_mma_y();

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
                    clamp(0, hit.range, 2.0), -hit.angle, is_hit,
                    path_to_send, pos_to_coord(k_goal_x, k_goal_y));
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


//////////////////////////
// SHARED FUNCTIONALITY //
//////////////////////////

// Gets the current approximate x position of the robot, accounting for error via an MMA
float get_current_mma_x() {
    if (!m_running_xs.size()) {
        return 0.0;
    }
    float running = 0.0;
    for (int i = 0; i < m_running_xs.size(); i++) {
        running += m_running_xs[i];
    }
    return running / m_running_xs.size();
}

// Gets the current approximate y position of the robot, accounting for error via an MMA
float get_current_mma_y() {
    if (!m_running_ys.size()) {
        return 0.0;
    }
    float running = 0.0;
    for (int i = 0; i < m_running_ys.size(); i++) {
        running += m_running_ys[i];
    }
    return running / m_running_ys.size();
}


/////////////////////////
// ASTAR FUNCTIONALITY //
/////////////////////////

// Perpetually astars and updates the path of the robot
void astar_thread() {
    bool finished = false;
    while (finished) {
        finished = astar(get_current_mma_x(), get_current_mma_y(), k_goal_x, k_goal_y);
    }
}

// Calculates astar using the current information
bool astar(float current_x, float current_y, float destination_x, float destination_y) {
    // Terminates when you are at the destination within a tolerance
    if (current_x < destination_x + k_end_tolerance
            && current_x > destination_x - k_end_tolerance){
        if (current_y < destination_y + k_end_tolerance
                && current_y > destination_y - k_end_tolerance){
            return true;
        }
    }

    // Creates node structures for the beginning of the search
    struct coord start_coord = pos_to_coord(current_x, current_y);
    struct coord end_coord = pos_to_coord(destination_x, destination_y);

    vector<node*> to_visit;

    node* first_node = make_node(start_coord);
    first_node->move_cost = 0;
    first_node->heur_cost = sqrt(pow(start_coord.x - end_coord.x, 2)
                                + pow(start_coord.y - end_coord.y, 2));
    to_visit.push_back(first_node);

    struct node *running;

    // Makes a safe copy of the running occupancy grid to perform A* on
    mut.lock();
    unordered_map<coord, cell_params> occupancy = occupancy_grid;
    mut.unlock();

    // Runs until out of nodes that need to be checked
    while (to_visit.size() > 0) {
        vector<node*>::iterator running_iterator = to_visit.begin();
        running = *running_iterator;

        // Checks all upcomming nodes to update 
        for (vector<node*>::iterator iter = to_visit.begin(); iter != to_visit.end(); iter++) {
            node* update_node = *iter;
            if (update_node->heur_cost + update_node->move_cost
                    <= running->move_cost + running->heur_cost) {
                running_iterator = iter;
                running = *running_iterator;
            }
        }

        // Removes the running node from the tasks
        to_visit.erase(running_iterator);

        // If we've reached the end_coord, we finna be done
        if (running->c == end_coord) {
            break;
        }

        // Gets all possible neighboring nodes to filter through
        vector<coord> to_check = neighbors(running->c);
        for (coord& c: to_check) {
            unordered_map<coord, cell_params>::iterator neib_it = occupancy.find(c);

            // *If* it's something we've seen...
            if (neib_it != occupancy.end()) {
                // If we know for sure it's a wall, then don't consider this
                struct cell_params checker = neib_it->second;
                if (is_wall(checker)) {
                    continue;
                }
            }

            // TODO - If this is slow, switch to a map to reduce runtime complexity
            node* visit_cell = nullptr;
            for (node* check : to_visit) {
                if (check->c == c) {
                    visit_cell = check;
                    break;
                }
            }

            // Calculates the possible updated running cost for this new cell
            float running_cost = running->move_cost;
            running_cost += sqrt(pow(c.x - running->c.x, 2) + pow(c.x - running->c.x, 2));

            // Updates cells if the total cost ends up being better
            if (visit_cell != nullptr) {
                // If this ends up being the running cheapest, update that bad boi
                if (running_cost < visit_cell->move_cost) {
                    visit_cell->parent_node = running;
                    visit_cell->move_cost = running_cost;
                }
            }
            // If the cell has not been visited yet by A*, add it to the queue
            else if (visit_cell == nullptr) {
                visit_cell = make_node(c);
                visit_cell->parent_node = running;
                visit_cell->move_cost = running_cost;
                visit_cell->heur_cost = sqrt(pow(c.x - end_coord.x, 2)
                                            + pow(c.y - end_coord.y, 2));
                to_visit.push_back(visit_cell);
            }
        }





    }
    
    // Runs backwards through the list to make a path of coordinates
    vector<coord> to_copy;
    while (!(running->c == start_coord)) {
        to_copy.push_back(running->c);
        running = running->parent_node;
    }

    // Copies the A* result to the shared memory
    mut.lock();
    m_path = to_copy;
    mut.unlock();

    // I hope this is how iterators work when freeing memory...
    vector<node*>::iterator visit_iter = to_visit.begin();
    while (visit_iter != to_visit.end()) {
        delete *visit_iter;
        visit_iter = to_visit.erase(visit_iter);
    }

    // You aint done yet dawg
    return false;
}


// Determines if a cell_params is describing a wall or not
bool is_wall(cell_params cp) {
    float wall_confidence = static_cast<float>(cp.num_hits * k_hit_priority)
                            / static_cast<float>(cp.num_hits * k_hit_priority + cp.num_misses); 
    return wall_confidence > k_wall_confidence;
}


// Makes a node struct
node* make_node(coord c) {
    node* ret = new node();
    ret->c = c;
    return ret;
}


// Makes a coord struct
coord make_coord(int x, int y) {
    struct coord c;
    c.x = x;
    c.y = y;
    return c;
}

// Gets the immediate neighboring coords of c
vector<coord> neighbors(coord c) {
    vector<coord> ret;
    ret.push_back(make_coord(c.x, c.y + 1));
    ret.push_back(make_coord(c.x, c.y - 1));
    ret.push_back(make_coord(c.x + 1, c.y));
    ret.push_back(make_coord(c.x - 1, c.y));
    ret.push_back(make_coord(c.x + 1, c.y + 1));
    ret.push_back(make_coord(c.x - 1, c.y - 1));
    ret.push_back(make_coord(c.x + 1, c.y - 1));
    ret.push_back(make_coord(c.x - 1, c.y + 1));
    return ret;
}


////////////////////
// MAIN EXECUTION //
////////////////////
int main(int argc, char* argv[]) {
    cout << "making robot" << endl;

    //XInitThreads();

    Robot robot(argc, argv, callback);
    cout << "ROBOT MADE" << endl;
    //thread rthr(robot_thread, &robot);
    //thread astar(astar_thread);

    viz_run();

    //rthr.join();
    //astar.join();

    return 0;
    //return viz_run();
}


// TODO: CHECK MUTEXES FOR OCCUPANCY GRID IN VIZ_HIT


