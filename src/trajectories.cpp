
#include "trajectories.h"

#include <cmath>

// define macros for threshold values and error values
#define THRESHOLD 0.05
#define ANGULAR_THRESHOLD 0.5
#define ANGULAR_ERROR 0.5


void goto_xy(robot_t& robot, float xt, float yt, float vt)
{
    float x = robot.xe;
    float y = robot.ye;
    float theta = robot.thetae;
    float v = robot.ve;
    float w = robot.we;

    float dx = xt - x;
    float dy = yt - y;
    
    // float rho = sqrt(dx*dx + dy*dy);
    float angle = atan2(dy, dx);
    // float beta = -theta - alpha;

    //error in angle
    robot.angular_error = angle - theta;

    normalize_angle(robot.angular_error);

    //velocity to reach the goal
    robot.dist_to_goal = sqrt(dx*dx + dy*dy);
    float v_goal = 0;
    //angular velocity to reach the goal
    float w_goal = 0;

    //create threshold for the goal
    if (fabs(robot.angular_error) > ANGULAR_THRESHOLD)
    {
        // v_goal = 0;
        w_goal = ANGULAR_ERROR * robot.angular_error;
    }
    else if (robot.dist_to_goal > THRESHOLD)
    {
        v_goal = vt * robot.dist_to_goal;
        if (v_goal > vt)
            v_goal = vt;
        w_goal = ANGULAR_ERROR * robot.angular_error;
    }
    else
    {
        v_goal = 0;
        w_goal = 0;
    }

    //update robot
    robot.v_req = v_goal;
    robot.w_req = w_goal;
}


void normalize_angle(float& angle)
{
    while(angle > M_PI)
    {
        angle -= 2*M_PI;
    }
    while(angle < -M_PI)
    {
        angle += 2*M_PI;
    }
}