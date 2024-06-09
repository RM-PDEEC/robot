#include <iostream>
#include <cmath>
#include <iomanip>
#include "trajectories.h"

using namespace std;

#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)

void simulate_follow_circle();

int main() {

    simulate_follow_circle();
    return 0;
}

void simulate_follow_circle() {
    traj.xr = 0.0;  // initial x position of the robot
    traj.yr = 0.0;  // initial y position of the robot
    traj.thetar = 0.0;  // orientation of the robot (in radians)
    traj.vt = 1.0;  // linear velocity
    traj.done = 0;
    traj.segments = 0;

    float xc = 0.0;  // x of circle center
    float yc = 0.0;  // y of circle center
    float rc = 0.5;  // radius
    float theta_f = (5 * M_PI)/4;  // final angle

    const int steps = 50;  // number of simulation steps
    const float time_step = 0.1;  // time step between epoch

    cout << fixed << setprecision(2);

    for (int i = 0; i < steps; ++i) {
        traj.follow_circle(xc, yc, rc, theta_f);

        // update robot position
        traj.xr += traj.v_req * cos(traj.thetar) * time_step;
        traj.yr += traj.v_req * sin(traj.thetar) * time_step;
        traj.thetar += traj.w_req * time_step;
        traj.thetar = normalize_angle(traj.thetar);

        cout << "Step " << i << ": ";
        cout << "xr = " << traj.xr << " ";
        cout << "yr = " << traj.yr << " ";
        cout << "thetar = " << traj.thetar << " ";
        cout << "v_req = " << traj.v_req << " ";
        cout << "w_req = " << traj.w_req << endl;
    }
}
