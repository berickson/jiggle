#include <iostream>
#include <iomanip>

#include <vector>
#include "math.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

using namespace std;

struct Pose {
    double x = NAN;
    double y = NAN;
    double theta = NAN;
};

struct PoseDelta {
    double r = NAN;
    double theta = NAN;
    double phi = NAN;
};

double distance(double x, double y) {
    return sqrt(x*x+y*y);
}


using namespace boost;

int main() {

    // help on properties
    // https://stackoverflow.com/a/7953988/383967
    typedef adjacency_list<vecS, vecS, bidirectionalS, Pose, PoseDelta > PoseGraph;
    PoseGraph g;

    vector <Pose> poses;
    vector <PoseDelta> deltas;

    add_edge(0, 1, PoseDelta{distance(2.5,5), atan2(2.5,5), 0}, g);
    add_edge(0, 1, PoseDelta{distance(2.4,5), atan2(2.4,5), 0}, g);
    add_edge(0, 2, PoseDelta{distance(4.4,2), atan2(4.4,2), 0}, g);
    add_edge(2, 1, PoseDelta{distance(3.1,2), atan2(-2,3.1), 0}, g);


    // Assign P0 a fixed pose estimate of {0, 0, 0}
    // All other poses have no estimates
    g[0] = Pose{0, 0, 0};
    // A pose is estimable if it shares an edge with an estimated pose - and isn't fixed (it could already have an estimate)

    // until converged:
    // for all estimable poses
    for(int pass = 0; pass< 10; ++pass) {
        for(int i = 1; i < num_vertices(g); ++i) {
            uint32_t n_estimates = 0;
            double sum_x = 0;
            double sum_y = 0;
            double sum_theta = 0;
            for(auto ep = in_edges(i, g);ep.first != ep.second; ++ep.first) {
                // edge to our edge
                PoseDelta & delta = g[*ep.first];
                Pose & from_pose = g[ep.first->m_source]; 
                if(from_pose.x != NAN) {
                    sum_x += from_pose.x + sin(from_pose.theta + delta.theta) * delta.r;
                    sum_y += from_pose.y + cos(from_pose.theta + delta.theta) * delta.r;
                    sum_theta += from_pose.theta + delta.phi;
                    ++n_estimates;
                }
            }
            if(n_estimates > 0) {
                // estimate = average calculated pose from connected poses
                // pose is now estimated

                Pose & pose = g[i];
                pose.x = sum_x / n_estimates;
                pose.y = sum_y / n_estimates;
                pose.theta = sum_theta / n_estimates;
            }
        }


        cout << "pass: " << pass << endl;
        // print_poses(poses);
        cout << "     x,     y, theta" << endl;
        for(int i = 0; i < num_vertices(g); ++ i) {
            Pose& pose = g[i];
            cout << std::setw(6)  << std::setprecision(4) << pose.x << "," << std::setw(6) << std::setprecision(4) << pose.y << "," << std::setw(6) << pose.theta << endl;
        }

        cout << endl;
    }

    return 0;
}