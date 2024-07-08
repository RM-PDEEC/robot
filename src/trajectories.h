#ifndef trajectories_H
#define trajectories_H

#include "Arduino.h"

class trajectory_t
{
  public:
    float xi, yi;
    float xt, yt;
    float vt;
    float v_nom, w_nom;
    float cx, cy;
    float e_xy, e_angle, e_c_angle, e_n;
    float xr, yr, thetar;

    float v_req, w_req;
    
    float done, segments;
    
    trajectory_t();
    
    void set_theta(void);
    void goto_xy(void);
    void follow_line(float xi_line, float yi_line, float xt_line, float yt_line);
    void follow_circle(float xc, float yc, float rc, float theta_f, int direction);
    void follow_segments(void);
};

float sqr(float x);
float dist(float x0, float y0, float x1, float y1);
float normalize_angle(float ang);
float dif_angle(float a0, float a1);

extern trajectory_t traj;

#endif // trajectories_H
