#ifndef GRID_HH
#define GRID_HH

#include "robot.hh"
#include "viz.hh"
#include "pose.hh"

static const float CELL_SIZE = 0.5;
static const float VIEW_SIZE = 41;

void grid_apply_hit(LaserHit hit, Pose pose);
Mat grid_view(Pose pose);
void grid_find_path(float x0, float y0, float x1, float y1);
float grid_goal_angle(Pose pose);

#endif
