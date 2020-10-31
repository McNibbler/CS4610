///////////////////////
// Thomas Kaunzinger //
//                   //
// CS4610            //
// Homework 7        //
///////////////////////

// Imports
#include <iostream>
#include <thread>
#include <mutex>
#include <math.h>
#include <deque>
#include <set>
#include <vector>
//#include <X11/Xlib.h>
//#undef Status
#include "robot.hh"
#include "viz.hh"

// Math macros
#define d2r(degrees) ((degrees) * M_PI / 180.0)
#define r2d(radians) ((radians) * 180.0 / M_PI)
#define clamp(min, x, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))


extern std::mutex mx;

// Clogged up namespace smh
using namespace std;

// Constants
#define k_goal_x                20.0    // Destination location
#define k_goal_y                0.0
#define k_end_tolerance         0.5
// Copied from viz.cc cus im lazy
#define k_width_meters          70.0
#define k_height_meters         70.0
#define k_window_height         1000
#define k_cell_size             3
#define k_granularity           (1.0 * (k_cell_size) / ((k_window_height) / ((k_height_meters)))) 
#define k_hit_priority          3       

#define k_wall                  1.5
#define k_far                   2.0
#define k_vel                   1.5
#define k_vel_fast              3.0
#define k_mma_window            10      // Window size of the modified moving average position
#define k_minimum_mma           5
#define k_discard               3       // How many of the first MMA values to discard
                                        // (because robo starts at 0,0 for some reason)
#define k_wall_confidence       0.75    // Scalar coefficient on confidence that a cell is a wall


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
vector<coord> m_last_path;

// Function initializations
void find_wall(Robot* robot);
void turn_right(Robot* robot);
void follow(Robot* robot);
bool is_wall(cell_params cp);
node make_node(coord c);
coord make_coord(int x, int y);
vector<coord> neighbors(coord c);
void astar_thread();
bool astar(float current_x, float current_y, float destination_x, float destination_y); 
float get_current_mma_x(); 
float get_current_mma_y(); 
float coord_dist(coord a, coord b);
int facing(float robot_degrees, float target_degrees, float tolerance);


/////////////////////////
// ROBOT FUNCTIONALITY //
/////////////////////////

// Robot callback
// Credit to The Construct on YouTube for the simple wallfollow implementation
void callback(Robot* robot) {

    if (robot->ranges.size() == 0) {
        return;
    }

    // float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = (clamp(0.0, robot->ranges.at(2).range, 2.05) + 
                 clamp(0.0, robot->ranges.at(2).range, 2.05) + 
                 clamp(0.0, robot->ranges.at(4).range, 2.05)) / 3.0;
    float frgt = (clamp(0.0, robot->ranges.at(0).range, 2.05) + 
                  clamp(0.0, robot->ranges.at(2).range, 2.05)) / 2.0;
    float flft = (clamp(0.0, robot->ranges.at(4).range, 2.05) + 
                  clamp(0.0, robot->ranges.at(5).range, 2.05)) / 2.0;


    // Speed is proportional to the distance from the wall (to avoid obstacles)
    int speed = k_vel * (fwd - k_wall);






    //if (m_state == FIRST_WALL && fwd > k_wall)                m_state = FIRST_WALL;
    //else if (fwd > k_wall && flft > k_wall && frgt > k_wall)  m_state = FIND_WALL;
    //else if (fwd < k_wall && frgt > k_wall && flft > k_wall)  m_state = TURN_RIGHT;
    //else if (fwd > k_wall && frgt > k_wall && flft < k_wall)  m_state = FOLLOW;
    //else if (fwd > k_wall && frgt < k_wall && flft > k_wall)  m_state = FIND_WALL;
    //else if (fwd < k_wall && frgt > k_wall && flft < k_wall)  m_state = TURN_RIGHT;
    //else if (fwd < k_wall && frgt < k_wall && flft > k_wall)  m_state = TURN_RIGHT;
    //else if (fwd < k_wall && frgt < k_wall && flft < k_wall)  m_state = TURN_RIGHT;
    //else if (fwd > k_wall && frgt < k_wall && flft < k_wall)  m_state = FIND_WALL;
    //else                                                      m_state = FIND_WALL;

    
    //// Determines what state to engage in
    //switch(m_state) {
    //    case FIRST_WALL:
    //        robot->set_vel(k_vel, k_vel);
    //        break;
    //    case FIND_WALL:
    //        //cout << "FIND WALL - FLFT<" << flft << ">, FWD<"
    //        //    << fwd << ">, FRGT<" << frgt << ">" << endl;
    //        find_wall(robot);
    //        break;
    //    case TURN_RIGHT:
    //        //cout << "TURN RIGHT - FLFT<" << flft << ">, FWD<"
    //        //    << fwd << ">, FRGT<" << frgt << ">" << endl;
    //        turn_right(robot);
    //        break;
    //    case FOLLOW:
    //        //cout << "FOLLOW - FLFT<" << flft << ">, FWD<"
    //        //    << fwd << ">, FRGT<" << frgt << ">" << endl;
    //        follow(robot);
    //        break;
    //    default:
    //        //cout << "DEFAULT" << endl;
    //        robot->set_vel(k_vel, k_vel);
    //}

    //cout << "x<" << robot->pos_x << ">, y<" << robot->pos_y
    //    << ">, t<" << r2d(robot->pos_t) << ">" << endl;


    // Updates the running x and y positions for the MMA algorithm
    mx.lock();

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

    // Copies the running path for the robo logic and the viz drawing
    vector<coord> path_to_send = m_path;

    // Gets the MMA
    float x_send = get_current_mma_x();
    float y_send = get_current_mma_y();

    mx.unlock();


    // Finds the coord that is the closest to the robot and then targets the next one
    float target_angle = 0;
    if (path_to_send.size() > 0) {

        // Searches for the closest node to the robot
        struct coord closest = path_to_send[0];
        struct coord robo_coord = pos_to_coord(x_send, y_send);
        float shortest_dist = coord_dist(closest, robo_coord);
        int index_best = 0;
        for (int i = 0; i < path_to_send.size(); i++) {
            struct coord c = path_to_send[i];
            float dist = coord_dist(c, robo_coord);
            if (dist < shortest_dist) {
                index_best = i;
                closest = c;
                shortest_dist = dist;
            }
        }

        // Picks the node next on the path after the closest node to the robot
        //int target_index = (index_best < path_to_send.size() - 1) ? target_index + 1 : target_index;
        struct coord target_coord = path_to_send[target_index];
        //struct coord target_coord = closest;

        // Finds the target angle between the robot and the coord
        target_angle = r2d(atan(static_cast<float>(target_coord.y - robo_coord.y)
                                / static_cast<float>(target_coord.x - robo_coord.x)));

        // Gazeboifies the angles
        while (target_angle > 180)      target_angle -= 360;
        while (target_angle < -180)     target_angle += 360;
    }


    // Turns in place until 15 degrees from target
    int face = facing(r2d(robot->pos_t), target_angle, 15);
    if (face == 0) {
        int check = facing(r2d(robot->pos_t), target_angle, 0.00001);
        // Moves in direction of the next angle
        (check > 0) ? robot->set_vel(speed * 1.2, speed * 0.8)
                    : robot->set_vel(speed * 0.8, speed * 1.2);
    }
    // Turn in place
    else {
        (face > 0) ? robot->set_vel(speed, -speed) : robot->set_vel(-speed, speed);
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
                    clamp(0, hit.range, 2.0), -hit.angle, is_hit,
                    path_to_send, pos_to_coord(k_goal_x, k_goal_y), m_last_path);
               // cout<<"["<<i<<"]: "<< hit.range<<"@"<<r2d(hit.angle)<<endl;
        }
    }
    m_last_x = x_send;
    m_last_y= y_send;

    m_last_path = path_to_send;
    usleep(250000);
}


// Inches forward to a wall
void find_wall(Robot* robot) {
    robot->set_vel(-k_vel, k_vel_fast); 
}

// 1 == too far left, 0 ==  facing, -1 == too far right
int facing(float robot_degrees, float target_degrees, float tolerance) {
    float diff = robot_degrees - target_degrees;
    diff = diff > 180 ? diff - 360 : diff;
    diff = diff < -180 ? diff + 360 : diff;
    if (abs(diff) < tolerance) {
        return 0;
    }
    int ret = (diff < 0) ? -1 : 1;
    return ret;
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

// Returns the distance between two coords
float coord_dist(coord a, coord b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

/////////////////////////
// ASTAR FUNCTIONALITY //
/////////////////////////

// Perpetually astars and updates the path of the robot
void astar_thread() {
    bool finished = false;
    // This is really epic and good
    while (get_current_mma_x() == 0 || get_current_mma_y() == 0) {
        sleep(1);
    }
    // Actually run A*
    while (!finished) {
        cout << "Running new instance of A*..." << endl;
        mx.lock();
        float x = get_current_mma_x();
        float y = get_current_mma_y();
        mx.unlock();
        finished = astar(x, y, k_goal_x, k_goal_y);
        sleep(1);
    }
}

// Calculates astar using the current information
bool astar(float current_x, float current_y, float destination_x, float destination_y) {

    // Gets bounds
    int max_x = static_cast<int>(ceil((k_width_meters / 2) / k_granularity));
    int max_y = static_cast<int>(ceil((k_height_meters / 2) / k_granularity));

    // Creates node structures for the beginning of the search
    struct coord start_coord = pos_to_coord(current_x, current_y);
    struct coord end_coord = pos_to_coord(destination_x, destination_y);

    if (start_coord == end_coord) {
        return true;
    }

    set<node> to_visit;
    //priority_queue<node*, std::greater<node*>> to_visit;
    unordered_map<coord, coord> seen;   // <child, parent>

    // The parent of the start node is itself
    seen[start_coord] = start_coord;

    node first_node = make_node(start_coord);
    first_node.parent_node = start_coord;
    first_node.move_cost = 0;
    first_node.heur_cost = sqrt(pow(start_coord.x - end_coord.x, 2)
                                + pow(start_coord.y - end_coord.y, 2));
    to_visit.insert(first_node);
    struct node running;

    // Makes a safe copy of the running occupancy grid to perform A* on
    mx.lock();
    unordered_map<coord, cell_params> occupancy = occupancy_grid;
    mx.unlock();

    // Runs until out of nodes that need to be checked
    while (to_visit.size() > 0) {
        //deque<node*>::iterator running_iterator = to_visit.begin();

        // Gets the lowest cost node in the visit set
        auto running_it = to_visit.begin();
        running = *running_it;

        // If we've reached the end_coord, we finna be done
        if (running.c == end_coord) {
            break;
        }

        // Removes the running node from the tasks
        seen[running.c] = running.parent_node;
        to_visit.erase(running_it);
        cout << running.c.x << ", " << running.c.y << ": " << to_visit.size() << endl;
        cout << running.move_cost + running.heur_cost << endl;

        // Gets all possible neighboring nodes to filter through
        vector<coord> neibs = neighbors(running.c);
        for (coord& c: neibs) {

            // If it's a wall, bad, skip
            unordered_map<coord, cell_params>::iterator neib_it = occupancy.find(c);
            // *If* it's something we've seen...
            float cost_multiplier = 1.0;
            if (neib_it != occupancy.end()) {
                // If we know for sure it's a wall, then don't consider this
                struct cell_params checker = neib_it->second;
                if (is_wall(checker)) {
                    cost_multiplier = 10000.0;
                    //continue;
                }
            }

            // Ignore seen nodes
            if (seen.find(c) != seen.end()) {
                continue;
            }

            // If OOB
            if (c.x > max_x || c.x < -max_x || c.y > max_y || c.y < -max_y) {
                continue;
            }

            // For every node in to_visit, if it is a neighbor of running, make that the cell the
            // Next to visit (break after -> this will be fast because it's ordered).
            struct node visit_cell = make_node(c);
            std::set<node>::iterator it = to_visit.find(visit_cell);


            // Calculates the possible updated running cost for this new cell
            float neib_cost = running.move_cost;
            neib_cost += sqrt(pow(c.x - running.c.x, 2) + pow(c.y - running.c.y, 2));

            // If the cell has not been visited yet by A*, add it to the queue
            if (it == to_visit.end()) {
                visit_cell = make_node(c);
                visit_cell.parent_node = running.c;
                visit_cell.move_cost = neib_cost;
                visit_cell.heur_cost = sqrt(pow(c.x - end_coord.x, 2)
                                            + pow(c.y - end_coord.y, 2))
                                            * cost_multiplier;
                // Adds!
                //if (neib_cost + coord_dist(c, end_coord) < running.move_cost + running.heur_cost) { // + visit_cell.heur_cost) {
                to_visit.insert(visit_cell);
                seen[visit_cell.c] = visit_cell.parent_node;
                //}
            }
            // Updates cells if the total cost ends up being better
            else {
                visit_cell = *it;
                // If this ends up being the running cheapest, update that bad boi
                if (neib_cost + coord_dist(c, end_coord) <  visit_cell.move_cost + visit_cell.heur_cost) {
                    visit_cell.parent_node = running.c;
                    visit_cell.move_cost = neib_cost;
                    // Updates!
                    to_visit.erase(it);
                    to_visit.insert(visit_cell);

                    seen[visit_cell.c] = visit_cell.parent_node;
                }
            }
        }
    }
    
    // Runs backwards through the list to make a path of coordinates
    vector<coord> to_copy;
    struct coord running_coord = running.c;
    while (!(running_coord == start_coord)) {
        to_copy.push_back(running_coord);
        running_coord = seen[running_coord];
    }

    // Copies the A* result to the shared memory
    mx.lock();
    m_path = to_copy;
    mx.unlock();

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
node make_node(coord c) {
    struct node ret; // = new node();
    ret.c = c;
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
    thread astar(astar_thread);

    viz_run();

    //rthr.join();
    astar.join();

    return 0;
    //return viz_run();
}


// TODO: CHECK mxEXES FOR OCCUPANCY GRID IN VIZ_HIT


