
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
#define ANGULAR_ERROR_LIMIT     0.5
#define ROTATION_STEP           0.5

// Function to perform 2D inverse transformation
Point2D inverseTransformPoint(const Point2D& world_coordinates, double tx, double ty, double angle);

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
  Point2D robot_coordinates = inverseTransformPoint({xt, yt}, 0, 0, 0);

  e_xy = dist(xr, yr, robot_coordinates.x, robot_coordinates.y);

  e_angle = dif_angle(atan2( robot_coordinates.y - yr,  robot_coordinates.x - xr), thetar);

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
  v_req = v_nom;
  w_req = 0;
}


void trajectory_t::follow_circle(void)
{
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
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transformation_matrix[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                transformation_matrix[i][j] += rotation_matrix[i][k] * translation_matrix[k][j];
            }
        }
    }

    // Transform coordinates
    Point2D robot_coordinates;
    robot_coordinates.x = world_coordinates.x * transformation_matrix[0][0] + world_coordinates.y * transformation_matrix[0][1] + transformation_matrix[0][2];
    robot_coordinates.y = world_coordinates.x * transformation_matrix[1][0] + world_coordinates.y * transformation_matrix[1][1] + transformation_matrix[1][2];
    
    return robot_coordinates;
}

