#include <iostream>
#include <iomanip>

#include <vector>
#include "math.h"

using namespace std;

struct Pose {
    double x = NAN;
    double y = NAN;
    double theta = NAN;
};

struct PoseDelta {
    uint32_t from;
    uint32_t to;
    double r = NAN;
    double theta = NAN;
    double phi = NAN;
};

double distance(double x, double y) {
    return sqrt(x*x+y*y);
}

void print_poses(const vector<Pose> & poses) {
    cout << "     x,     y, theta" << endl;
    for(auto & pose : poses) {
        cout << std::setw(6) << pose.x << "," << std::setw(6) << pose.y << "," << std::setw(6) << pose.theta << endl;
    }
}



int main() {
    vector <Pose> poses;
    vector <PoseDelta> deltas;

    poses.resize(3);
    deltas = {
        {0, 1, distance(2.5,5), atan2(2.5,5), 0},
        {0, 2, distance(4.4,2), atan2(4.4,2), 0},
        {2, 1, distance(3.1,2), atan2(-2,3.1), 0}
    };

    // Assign P0 a fixed pose estimate of {0, 0, 0}
    // All other poses have no estimates
    poses[0] = {0, 0, 0};
    // A pose is estimable if it shares an edge with an estimated pose - and isn't fixed (it could already have an estimate)

    // until converged:
    // for all estimable poses
    for(int pass = 0; pass< 10; ++pass) {
        for(int i = 1; i < poses.size(); ++i) {
            uint32_t n_estimates = 0;
            double sum_x = 0;
            double sum_y = 0;
            double sum_theta = 0;
            for(auto & delta : deltas) {
                // edge to our edge
                if(delta.to == i) {
                    Pose & from_pose = poses[delta.from];
                    if(from_pose.x != NAN) {
                        sum_x += from_pose.x + sin(from_pose.theta + delta.theta) * delta.r;
                        sum_y += from_pose.y + cos(from_pose.theta + delta.theta) * delta.r;
                        sum_theta += from_pose.theta + delta.phi;
                        ++n_estimates;
                    }
                }
            }
            if(n_estimates > 0) {
                // estimate = average calculated pose from connected poses
                // pose is now estimated

                Pose & pose = poses[i];
                pose.x = sum_x / n_estimates;
                pose.y = sum_y / n_estimates;
                pose.theta = sum_theta / n_estimates;
            }
        }


        cout << "pass: " << pass << endl;
        print_poses(poses);
        cout << endl;
    }

    return 0;
}