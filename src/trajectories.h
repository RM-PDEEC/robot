#ifndef trajectories_H
#define trajectories_H

#include "Arduino.h"
#include "robot.h"

void normalize_angle(float& angle);

void goto_xy(robot_t& robot, float xt, float yt, float vt);

#endif // trajectories_H
