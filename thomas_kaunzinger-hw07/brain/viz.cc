/*
A simple example of using the gfx library.
CSE 20211
9/7/2011
by Prof. Thain

Modified by Nat Tuck, Oct 2020
Also modified by Thomas Kaunzinger
*/

// Imports
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <mutex>
#include <iostream>

#include <unordered_map>
#include <vector>
#include "viz.hh"

extern "C" {
#include "gfx.h"
}

// Math macros
#define d2r(degrees) ((degrees) * M_PI / 180.0)
#define r2d(radians) ((radians) * 180.0 / M_PI)
#define clamp(min, x, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

// Constants
#define k_width_meters      70
#define k_height_meters     70
#define k_window_height     1000
#define k_window_width      1000
#define k_cell_size         3
#define k_granularity       (1.0 * (k_cell_size) / ((k_window_height) / ((k_height_meters))))
#define k_bg_r              224
#define k_bg_g              224
#define k_bg_b              255
#define k_hit_priority      3

// Clogged up namespace
using namespace std;

// Function instantiation
float diff(float deg1, float deg2);
vector<coord> bresenham(coord pos1, coord pos2);
void draw_square(int x, int y, int width);

// Init stuff
typedef lock_guard<mutex> guard;
mutex mx;
static int viz_init = 0;


// Something is hit -> update map
int viz_hit(unordered_map<coord, cell_params>& occupancy_grid,
            float robot_x, float robot_y, float robot_t,
            float prev_x, float prev_y,
            float range, float angle, bool is_hit,
            vector<coord> path, coord end_coord) {
    guard _gg(mx);
    if (!viz_init) {
      puts("viz skip");
      return 0;
    }
  
  
    struct coord robot_pos;
    robot_pos.x = static_cast<int>(round(robot_x / k_granularity));
    robot_pos.y = static_cast<int>(round(robot_y / k_granularity));
    float current_angle = r2d(robot_t);

    float actual_angle = diff(r2d(angle), current_angle);
    float relative_x = 0;
    float relative_y = 0;

    //             90   +y
    //                |
    //                |   
    //              2 | 1
    // +/-180 --------|-------- 0
    //   -x         3 | 4       +x
    //                |    
    //                |
    //            -90   -y
    relative_x = cos(d2r(actual_angle)) * range;
    relative_y = -1 * sin(d2r(actual_angle)) * range;

    struct coord hit_pos;
    hit_pos.x = static_cast<int>(round((robot_x + relative_x) / k_granularity));
    hit_pos.y = static_cast<int>(round((robot_y + relative_y) / k_granularity));
    
    unordered_map<coord, cell_params>::iterator it = occupancy_grid.find(hit_pos);
    
    struct cell_params hit_cell;
    // Create a new entry
    if (it == occupancy_grid.end()) {
        if (is_hit) {
            hit_cell.num_hits = 1;
            hit_cell.num_misses = 0;
        }
        else {
            hit_cell.num_hits = 0;
            hit_cell.num_misses = 1;
        }
        hit_cell.last_updated = 0;    // NOTE: Update this for time decay
        mx.lock();
        occupancy_grid[hit_pos] = hit_cell;
        mx.unlock();
    }
    // Update existing entry
    else {
        hit_cell = it->second;
        if (is_hit) {
            hit_cell.num_hits++;
        }
        else {
            hit_cell.num_misses++;
        }
        mx.lock();
        occupancy_grid[hit_pos] = hit_cell;        
        mx.unlock();
    }

    int width = k_window_width;
    int height = k_window_height;
    int center_x, center_y;
    center_x = static_cast<int>(width / 2);
    center_y = static_cast<int>(height / 2);

    // Gets every point along the "hit" line that didn't actually hit
    vector<coord> vacant_points = bresenham(robot_pos, hit_pos);
   
    for (coord& c: vacant_points) {
        
        unordered_map<coord, cell_params>::iterator vacant_it = occupancy_grid.find(c);
        struct cell_params update_cell;

        if (vacant_it == occupancy_grid.end()) {
            update_cell.num_hits = 0;
            update_cell.num_misses = 1;
            update_cell.last_updated = 0;    // NOTE: Update this for time decay
            mx.lock();
            occupancy_grid[c] = update_cell;
            mx.unlock();
        }
        // Update existing entry
        else {
            update_cell = vacant_it->second;
            update_cell.num_misses++;
            mx.lock();
            occupancy_grid[c] = update_cell;        
            mx.unlock();
        }

        int light = 255 - static_cast<int>((255 * update_cell.num_hits * k_hit_priority)
                                / (update_cell.num_hits * k_hit_priority + update_cell.num_misses)); 
        gfx_color(light, light, light);
        //gfx_point(center_x + c.x, center_y - c.y);
        draw_square(center_x + c.x * k_cell_size, center_y - c.y * k_cell_size, k_cell_size);
    }

    // Adds the hit cell
    int light = 255 - static_cast<int>((255 * hit_cell.num_hits * k_hit_priority)
                            / (hit_cell.num_hits * k_hit_priority + hit_cell.num_misses)); 
    gfx_color(light, light, light);
    //gfx_point(center_x + hit_pos.x, center_y - hit_pos.y);
    draw_square(center_x + hit_pos.x * k_cell_size, center_y - hit_pos.y * k_cell_size, k_cell_size);


    // Undraws the previous robot position
    struct coord prev;
    prev.x = static_cast<int>(round(prev_x / k_granularity));
    prev.y = static_cast<int>(round(prev_y / k_granularity));
    auto prev_check = occupancy_grid.find(prev);
    if (prev_check == occupancy_grid.end()) {
        gfx_color(k_bg_r, k_bg_g, k_bg_b);
    }
    else {
        struct cell_params c = prev_check->second;
        int prev_color = 255 - static_cast<int>((255 * c.num_hits * k_hit_priority)
                                / (c.num_hits * k_hit_priority + c.num_misses)); 
        gfx_color(prev_color, prev_color, prev_color);
    }
    draw_square(center_x + prev.x * k_cell_size, center_y - prev.y * k_cell_size, k_cell_size);

    // Draws the path
    if (path.size() > 0) {
        gfx_color(128, 100, 192);
        for (coord& c: path) {
            draw_square(center_x + c.x * k_cell_size,
                        center_y - c.y * k_cell_size, k_cell_size);
        }
    }

    // Draws the goal
    gfx_color(128, 192, 128);
    draw_square(center_x + end_coord.x * k_cell_size,
                center_y - end_coord.y * k_cell_size, k_cell_size);

    // Draws the new robot position
    gfx_color(0, 128, 192);
    draw_square(center_x + robot_pos.x * k_cell_size,
                center_y - robot_pos.y * k_cell_size, k_cell_size);


    //gfx_flush();
  
    return 0;
}





// Draws a square at the given location
void draw_square(int start_x, int start_y, int width) {
    for (int y = 0; y < width; y++) {
        for (int x = 0; x < width; x++) {
            gfx_point(start_x + x, start_y + y);
        }
    }
}


// Gets difference between two angles on a circle
float diff(float deg1, float deg2) {
    float d = deg1 - deg2;
    d = d > 180 ? d - 360 : d;
    d = d < -180 ? d + 360 : d;
    return d;
}


// Calculates the empty points in a line between two points using bresenham's algorithm
// Implementation inspired by MyTechnoWorld
vector<coord> bresenham(coord pos1, coord pos2) {
    int x = pos1.x;
    int y = pos1.y;
    int x2 = pos2.x;
    int y2 = pos2.y;

    int dx, dy, p;
    dx = x2 - x;
    dy = y2 - y;
    p = 2 * (dy) - (dx);

    vector<coord> ret = {};

    int w = x2 - x ;
    int h = y2 - y ;
    int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0 ;
    if (w<0) dx1 = -1 ; else if (w>0) dx1 = 1 ;
    if (h<0) dy1 = -1 ; else if (h>0) dy1 = 1 ;
    if (w<0) dx2 = -1 ; else if (w>0) dx2 = 1 ;
    int longest = abs(w) ;
    int shortest = abs(h) ;
    if (!(longest > shortest)) {
        longest = abs(h) ;
        shortest = abs(w) ;
        if (h<0) dy2 = -1 ; else if (h>0) dy2 = 1 ;
        dx2 = 0 ;            
    }
    int numerator = longest >> 1 ;
    for (int i=0;i<=longest;i++) {
        struct coord to_add;
        to_add.x = x;
        to_add.y = y;
        ret.push_back(to_add);
        numerator += shortest ;
        if (!(numerator<longest)) {
            numerator -= longest ;
            x += dx1 ;
            y += dy1 ;
        } else {
            x += dx2 ;
            y += dy2 ;
        }
    }
    return ret;
}
    

// Converts a position to a coord on the occupancy map
coord pos_to_coord(float x, float y) {
    struct coord c;
    c.x = static_cast<int>(round(x / k_granularity));
    c.y = static_cast<int>(round(y / k_granularity));
    return c;
}

// Convers a coord on the occupancy map to an approximate position
pair<float, float> coord_to_pos(coord c) {
    float x, y;
    x = static_cast<float>(c.x * k_granularity);
    y = static_cast<float>(c.y * k_granularity);
    pair<float, float> ret(x, y);
    return ret;
}






// Initialization
void 
viz_run()
{

  std::cout << "test" << std::endl;

  int ysize = k_window_height;
  int xsize = k_window_width;

  char c;

  // Open a new window for drawing.
  gfx_open(xsize,ysize,"Example Graphics Program");

  // Sets the background color
  gfx_clear_color(k_bg_r, k_bg_g, k_bg_b);
  gfx_clear();

  // Set the current drawing color to green.
  //gfx_color(0,200,100);

  // // Draw a triangle on the screen.
  // gfx_line(100,100,200,100);
  // gfx_line(200,100,150,150);
  // gfx_line(150,150,100,100);

  {
    guard _gg(mx);
    viz_init = 1;
  }
  
	while(1) {
		// Wait for the user to press a character.
		c = gfx_poll();

		// Quit if it is the letter q.
		if(c=='q') break;
    usleep(50000);
	}
}
