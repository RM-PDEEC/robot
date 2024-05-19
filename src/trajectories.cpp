
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

#define DISTANCE_ERROR_LIMIT    0.05
#define ANGULAR_ERROR_LIMIT     0.1
#define ROTATION_STEP           2.1
#define NORMAL_STEP             3.5

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
    w_nom = ROTATION_STEP * e_angle;
    v_nom = 0;
  }
  else if (e_xy > DISTANCE_ERROR_LIMIT)
  {
    w_nom = ROTATION_STEP * e_angle;
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

void trajectory_t::follow_line(void)
{
    // get the robot coordinates in the world frame
    // Point2D target_robot_coordinates = inverseTransformPoint({xt, yt}, 0, 0, 0);

    // e_xy = dist(xr, yr, world_coordinates.x, world_coordinates.y);

    // e_angle = dif_angle(atan2( world_coordinates.y - yr,  world_coordinates.x - xr), thetar);

    // line paralele to y axis
    float xi_line = 0.0;
    float yi_line = 0.30;
    float xt_line = 1.0;
    float yt_line = 0.30;

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
      w_nom = ROTATION_STEP * e_angle + e_n * NORMAL_STEP;
      v_nom = vt * cos(e_angle);
      if (v_nom > vt)
        v_nom = vt;
    }
    else
    {
      w_nom = 0;
      v_nom = 0;
    }
 

    v_req = v_nom;
    w_req = w_nom;
}


void trajectory_t::follow_circle(void)
{
  //circle diameter 1
  float xc = 0.0; // x center
  float yc = -0.5; // y center
  float rc = 0.5; // radius

  // // calculate the tangent to the circle in the closest point to the robot
  float d = dist(xr, yr, xc, yc);
  e_n = d - rc;
  float alpha = atan2(yc - yr, xc - xr); // orientation to the center of the circle

  // normalize the angle to the range of [-π, π]
  alpha = normalize_angle(alpha);

  // calculate the angle perpendicular to alpha to get the tangent
  float theta; 
  // float positive = 1;
  if (yc >= 0)
  {
    theta = alpha - PI / 2;
  }
  else
  {
    theta = alpha + PI / 2;
  }


  e_angle = dif_angle(theta, thetar);

  // define the final point of the arc trajectory depending on the center of the circle for x and y
  float xt_circle = xc * 2;
  float yt_circle = yc * 2;
  
  //calculate the distance from the robot to the target
  e_xy = dist(xr, yr, xt_circle, yt_circle);


  if (e_xy > DISTANCE_ERROR_LIMIT)
  {
    if (e_angle > 0)
    {
      w_nom = ROTATION_STEP * e_angle + e_n * NORMAL_STEP;
      v_nom = vt * cos(e_angle);
      if (v_nom > vt)
        v_nom = vt;
    }
    else
    {
      w_nom = ROTATION_STEP * e_angle - e_n * NORMAL_STEP;
      v_nom = vt * cos(e_angle);
      if (v_nom > vt)
        v_nom = vt;
    }
    // w_nom = ROTATION_STEP * e_angle + e_n * NORMAL_STEP;
    // v_nom = vt * cos(e_angle);
    // if (v_nom > vt)
    //   v_nom = vt;
  }
  else
  {
    w_nom = 0;
    v_nom = 0;
  }

  v_req = v_nom;
  w_req = w_nom;
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