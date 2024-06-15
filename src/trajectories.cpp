
#include "trajectories.h"

#include <iostream>
#include <cmath>
#include <array>


trajectory_t traj;
// Define a 2D point structure
struct Point2D {
    double x;
    double y;
};

#define DISTANCE_ERROR_LIMIT    0.045
#define ANGULAR_ERROR_LIMIT     0.17
#define ROTATION_STEP_LINE      2.1
#define NORMAL_STEP_LINE        4.5
#define ROTATION_STEP_CIRC      0
#define NORMAL_STEP_CIRC        0

// Function to perform 2D inverse transformation
Point2D inverseTransformPoint(const Point2D& world_coordinates, double tx, double ty, double angle);
// Function to perform 2D transformation
Point2D transformPoint(const Point2D& robot_coordinates, double tx, double ty, double angle);

float sqr(float x)
{
  return x * x;
}


float dist(float x0, float y0, float x1, float y1)
{
  return sqrt(sqr(x1 - x0) + sqr(y1 - y0));
}

// Normalize angle to the range of [-π, π]
float normalize_angle(float angle)
{
  if (angle >= 0) {
    angle = fmod(angle + PI, TWO_PI);
    return angle - PI;
  } else {
    angle = fmod(-angle + PI, TWO_PI);
    return -(angle - PI);
  }}


float dif_angle(float a0, float a1)
{
  return normalize_angle(normalize_angle(a0) - normalize_angle(a1));
}


trajectory_t::trajectory_t()
{

}

void trajectory_t::set_theta(void)
{
  v_req = 0;
  w_req = 0;
}

void trajectory_t::goto_xy(void)
{
  Point2D target_robot_coordinates = inverseTransformPoint({xt, yt}, 0, 0, 0);

  e_xy = dist(xr, yr, target_robot_coordinates.x, target_robot_coordinates.y);

  e_angle = dif_angle(atan2( target_robot_coordinates.y - yr,  target_robot_coordinates.x - xr), thetar);

  if (fabs(e_angle) > ANGULAR_ERROR_LIMIT)
  {
    w_nom = ROTATION_STEP_LINE * e_angle;
    v_nom = 0;
  }
  else if (e_xy > DISTANCE_ERROR_LIMIT)
  {
    w_nom = ROTATION_STEP_LINE * e_angle;
    v_nom = vt * cos(e_angle) * e_xy;
    if (v_nom > vt)
      v_nom = vt * cos(e_angle);
  }
  else
  {
    w_nom = 0;
    v_nom = 0;
  }

  w_req = w_nom;
  v_req = v_nom;
}

void trajectory_t::follow_line(float xi_line, float yi_line, float xt_line, float yt_line)
{
    // get the robot coordinates in the world frame
    // Point2D target_robot_coordinates = inverseTransformPoint({xt, yt}, 0, 0, 0);

    // line paralele to y axis
    // float xi_line = 0.0;
    // float yi_line = 0.30;
    // float xt_line = 1.0;
    // float yt_line = 0.30;

    // line paralele to x axis
    // float yi_line = 0.0;
    // float xi_line = 0.30;
    // float yt_line = 1.0;
    // float xt_line = 0.30;

    //caluclate the module of the vector from the robot and perpendicular to the closest point in the line using the escalar of the vectors
    float module = ((xr - xi_line) * (xt_line - xi_line) + (yr - yi_line) * (yt_line - yi_line)) / (sqr(xt_line - xi_line) + sqr(yt_line - yi_line));
    //calculate the point in the line
    cx = xi_line + module * (xt_line - xi_line);
    cy = yi_line + module * (yt_line - yi_line);

    //calculate the distance from the robot to the line
    e_n = dist(xr, yr, cx, cy);

    //calculate the angle between the robot and the line
    // e_c_angle = dif_angle(atan2(cy - yr, cx - xr), thetar);

    //calculate the angle between the robot and the target
    e_angle = dif_angle(atan2(yt_line - yr, xt_line - xr), thetar);

    //calculate the distance from the robot to the target
    e_xy = dist(xr, yr, xt_line, yt_line);
    
    if (e_xy > DISTANCE_ERROR_LIMIT)
    {
      w_nom = ROTATION_STEP_LINE * e_angle + e_n * NORMAL_STEP_LINE;
      v_nom = vt * cos(e_angle);
      if (v_nom > vt)
        v_nom = vt;
    }
    else
    {
      w_nom = 0;
      v_nom = 0;
      done = 1;
    }
 
    v_req = v_nom;
    w_req = w_nom;
}


void trajectory_t::follow_circle(float xc, float yc, float rc, float theta_f)
{
  //circle diameter 1
  // float xc = 0.0; // x center
  // float yc = -0.5; // y center
  // float rc = 0.5; // radius

  // // calculate the tangent to the circle in the closest point to the robot
  float d = dist(xr, yr, xc, yc);
  e_n = d - rc;
  float alpha = atan2(yr - yc, xr - xc); // orientation to the center of the circle

  // normalize the angle to the range of [-π, π]
  alpha = normalize_angle(alpha);

  // calculate the angle perpendicular to alpha to get the tangent
  float theta; 
  // float positive = 1;

  // Determine the sign of the angle based on the orientation of the robot
  if (theta_f >= 0)
  {
    theta = alpha + PI / 2;
  }
  else
  {
    theta = alpha - PI / 2;
  }

  // else
  // {
  //   if (theta_f >= 0)
  //   {
  //     theta = alpha - PI / 2;
  //   }
  //   else
  //   {
  //     theta = alpha + PI / 2;
  //   }
  // }

  theta = normalize_angle(theta);

  e_angle = dif_angle(theta, thetar);


  // float ro = atan2(yi - yc, xi - xc); // inicial angle from the center of the circle to the initial point
  // float beta = atan2(yr - yc, xr - xc); // angle from the center of the circle to the robot
  // float t_angle = dif_angle(beta, ro); // angle from the initial point to the robot
  float xt_circle = xc + rc * cos(theta_f);
  float yt_circle = yc + rc * sin(theta_f);
  
  //calculate the distance from the robot to the target
  e_xy = dist(xr, yr, xt_circle, yt_circle);


  if (e_xy > DISTANCE_ERROR_LIMIT) // || t_angle > ANGULAR_ERROR_LIMIT)
  {
    if (e_angle > 0)
    {
      w_nom = ROTATION_STEP_CIRC * e_angle + e_n * NORMAL_STEP_CIRC + vt/rc;
      if(e_angle > PI/2)
      {
        v_nom = 0;
      }
      else {
      v_nom = vt * cos(e_angle);
      }
      if (v_nom > vt)
        v_nom = vt;
    }
    else
    {
      w_nom = ROTATION_STEP_CIRC * e_angle - e_n * NORMAL_STEP_CIRC - vt/rc;
      v_nom = vt * cos(e_angle);
      if(e_angle < -PI/2)
      {
        v_nom = 0;
      }
      else {
      v_nom = vt * cos(e_angle);
      }
      if (v_nom > vt)
        v_nom = vt;
    }
  }
  else
  {
    w_nom = 0;
    v_nom = 0;
    done = 1;
  }

  v_req = v_nom;
  w_req = w_nom;
}

void trajectory_t::follow_segments(void)
{
  if (done == 1)
  {
    segments += 1;
    done = 0;
  }
  if (segments == 0)
  {
    follow_line(0.0, 0.15, 0.3, 0.15);
  }
  else if (segments == 1)
  {
    follow_circle(0.3, 0.0, 0.15, -PI/2.0);
  }
  else if (segments == 2)
  {
    follow_line(0.3, -0.15, 0.0, -0.15);
  }
  else if (segments == 3)
  {
    follow_circle(0.0, 0.0, 0.15, PI/2);
  }
  else
  {
    done = 1;
  }
}


// Function to perform 2D inverse transformation
Point2D inverseTransformPoint(const Point2D& world_coordinates, double tx, double ty, double angle) {
  // Convert angle to radians
  double theta = angle * M_PI / 180.0; // Negative of angle for inverse transformation

  // Construct translation matrix
  std::array<std::array<double, 3>, 3> translation_matrix = {{
    {1, 0, tx}, // Negative of translation for inverse transformation
    {0, 1, ty},
    {0, 0, 1}
  }};

  // Construct rotation matrix
  std::array<std::array<double, 3>, 3> rotation_matrix = {{
    {cos(theta), -sin(theta), 0},
    {sin(theta), cos(theta), 0},
    {0, 0, 1}
  }};

  // Multiply rotation and translation matrices
  std::array<std::array<double, 3>, 3> transformation_matrix;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      transformation_matrix[i][j] = 0;
      for (int k = 0; k < 3; ++k)
      {
        transformation_matrix[i][j] += rotation_matrix[i][k] * translation_matrix[k][j];
      }
    }
  }

  // Transform coordinates
  Point2D robot_world_coordinates;
  robot_world_coordinates.x = world_coordinates.x * transformation_matrix[0][0] + world_coordinates.y * transformation_matrix[0][1] + transformation_matrix[0][2];
  robot_world_coordinates.y = world_coordinates.x * transformation_matrix[1][0] + world_coordinates.y * transformation_matrix[1][1] + transformation_matrix[1][2];
  
  return robot_world_coordinates;
}

// Function to perform 2D transformation
Point2D transformPoint(const Point2D& robot_coordinates, double tx, double ty, double angle) {
  // Convert angle to radians
  double theta = angle * M_PI / 180.0;

  // Construct translation matrix
  std::array<std::array<double, 3>, 3> translation_matrix = {{
    {1, 0, tx},
    {0, 1, ty},
    {0, 0, 1}
  }};

  // Construct rotation matrix
  std::array<std::array<double, 3>, 3> rotation_matrix = {{
    {cos(theta), -sin(theta), 0},
    {sin(theta), cos(theta), 0},
    {0, 0, 1}
  }};

  // Multiply translation and rotation matrices
  std::array<std::array<double, 3>, 3> transformation_matrix;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      transformation_matrix[i][j] = 0;
      for (int k = 0; k < 3; ++k)
      {
        transformation_matrix[i][j] += translation_matrix[i][k] * rotation_matrix[k][j];
      }
    }
  }

  // Transform coordinates
  Point2D world_coordinates;
  world_coordinates.x = robot_coordinates.x * transformation_matrix[0][0] + robot_coordinates.y * transformation_matrix[0][1] + transformation_matrix[0][2];
  world_coordinates.y = robot_coordinates.x * transformation_matrix[1][0] + robot_coordinates.y * transformation_matrix[1][1] + transformation_matrix[1][2];
  
  return world_coordinates;
}