#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <iomanip>
#include <vector>
#include <functional>
#include <chrono>
#include <random>

//#include <angles/angles.h>
namespace angles {
    float normalize_angle(float radians) {
        while(radians > M_PI) {
            radians -= 2 * M_PI;
        }
        while(radians < -M_PI) {
            radians += 2 * M_PI;
        }
        return radians;
    }
}

#define degrees2radians(theta) ((theta) * EIGEN_PI / 180.)
#define radians2degrees(theta) ((theta) * 180. / EIGEN_PI)

size_t g_scan_difference_count = 0;

std::default_random_engine random_engine;

using namespace std;
using namespace std::chrono;

template <class T>
struct Point2d {
    Point2d() {
        x = NAN;
        y = NAN;
    }
    Point2d(T x, T y) : x(x), y(y) {}
    T x;
    T y;
    float norm() const {
        return sqrt(x*x+y*y);
    }
};


class Stopwatch {
public:
    size_t start_count = 0;
    time_point<system_clock> start_time;
    duration<int64_t, std::ratio<1,1000000000>> elapsed_time = duration<int64_t, std::ratio<1,1000000000>>::zero();
    bool started = false;
    void start() {
        ++start_count;
        start_time = system_clock::now();
        started = true;
    }

    void stop() {
        if(started) {
            elapsed_time += system_clock::now() - start_time;
            started = false;
        }
    }

    double get_elapsed_seconds() {
        auto elapsed = started ? elapsed_time + system_clock::now() - start_time : elapsed_time; 
        return duration_cast<duration<double>>(elapsed_time).count();
    }
};

template <class T=float>
struct Polar {
    T r;
    T theta;
};

template <class T=float>
class Pose
{
private:
    T x;
    T y;
    T theta;
    T cos_theta;
    T sin_theta;

public:
    inline T get_x() const { return x; }
    inline T get_y()  const { return y; }
    inline T get_theta()  const { 
        return theta; 
    }
    inline Polar<T> get_polar() const {
        Polar<T> p;
        p.r = sqrt(x*x+y*y);
        p.theta = angles::normalize_angle(atan2(y,x));
        return p;
    }

    void update_trig() {
        cos_theta = cos(theta);
        sin_theta = sin(theta);
    }

    Pose(T x = 0.0, T y=0.0, T theta = 0.0) : x(x), y(y), theta(theta) {
        update_trig();
    }

    void move(Point2d<T> p, T dtheta) {
        Point2d<T> w = Pose2World(p);
        x = w.x;
        y = w.y;
        theta += dtheta;
        theta = angles::normalize_angle(theta);

        update_trig();
    }

    // returns pose from this pose to p2.  p2-p1.
    Pose relative_pose_to(Pose p2) {
        // calculate world difference
        Pose & p1 = *this;
        T dx = p2.x - p1.x;
        T dy = p2.y - p1.y;
        T theta = angles::normalize_angle(atan2(dy,dx));
        T r = sqrt(dy*dy+dx*dx);

        // Pose rv in terms of p1
        T theta_rel = theta-p1.theta;
        Pose rv;
        rv.x = cos(theta_rel)*r;
        rv.y = sin(theta_rel)*r;
        rv.theta = angles::normalize_angle(p2.theta - p1.theta);
        rv.update_trig();
        return rv;
        

    }

    inline Point2d<T>  Pose2World(const Point2d<T> & p) const {
        Point2d<T> rv;
        rv.x = p.x * cos_theta - p.y * sin_theta + x;
        rv.y = p.x * sin_theta + p.y * cos_theta + y;
        return rv;
    }


    Eigen::Transform<T,2,Eigen::Affine> Pose2WorldTransform() const {
        Eigen::Transform<T,2,Eigen::Affine> t;
        t.setIdentity();
        t.rotate(theta);
        t.translate( Vector2T(x,y));
        return t;
    }

    Eigen::Transform<T,2,Eigen::Affine> World2PoseTransform() const {
        return Pose2WorldTransform().inverse();
    }

};


template <class T = double>
struct ScanLine {
    ScanLine(T theta = NAN, T d = NAN) : theta(theta), d(d) {
    }

    T theta = NAN;
    T d = NAN;
};


template <class T>
std::string to_sring(Point2d<T> & p) {
    stringstream ss;
    ss << "(" << p.x << ", " << p.y << ")";
    return ss.str();
}

template <class T>
std::string to_string(Pose<T> & pose) {
    stringstream ss;
    ss << std::fixed << std::setprecision(3) << pose.get_x() 
       << ", " << pose.get_y() 
       << ", " << std::setprecision(1) << radians2degrees(angles::normalize_angle(pose.get_theta())) << "°";
    return ss.str();
}

template <class T>
void print_scan(vector<ScanLine<T>> scan) {
    cout << "degrees, d, px, py" << endl;
    for(auto s : scan) {
        T theta = s.theta;
        T px = s.d*cos(theta);
        T py = s.d*sin(theta);
        cout << radians2degrees(s.theta) << ", " << s.d << ", " << px << ", " << py << endl;
    }
}

template <class T = double>
inline bool is_between(T a, T b, T c) {
    return (b <= a && a <=c) || (c <= a && a <= b);
}

template <class T = double>
inline bool is_between(T a, T b, T c, T e) {
    return (b-e <= a && a <=c+e) || (c-e <= a && a <= b+e);
}

template <class T = double>
class Line {
    typedef Eigen::Matrix<T,2,1> Vector2T;
    typedef Eigen::Matrix<T,3,1> Vector3T;
 public:
    T a;
    T b;
    T c;

   static Line<T> from_points(const Point2d<T> & p1, const Point2d<T> & p2) {
       auto h1 = Vector3T(p1.x,p1.y,1);      
       auto h2 = Vector3T(p2.x,p2.y,1);
       auto l = h1.cross(h2);
       Line line;
       line.a = l(0);
       line.b = l(1);
       line.c = l(2);
       return line;  
    }

    Point2d<T> intersection(Line l2) {
        auto h = Vector3T(a,b,c).cross(Vector3T(l2.a, l2.b, l2.c));
        if(h(2)==0) {
            return {NAN, NAN};
        }
        return {h(0)/h(2), h(1)/h(2)};
    }
};

template <class T=double>
class LineSegment {
    typedef Eigen::Matrix<T,2,1> Vector2T;
    public:
    Point2d<T> p1;
    Point2d<T> p2;
    LineSegment(T x1, T y1, T x2, T y2) : p1(x1, y1), p2(x2, y2) {      
    }
    LineSegment(Point2d<T> p1, Point2d<T> p2) : p1(p1), p2(p2) {      
    }

    Point2d<T> intersection(LineSegment & s2) {
        auto l2 = Line<T>::from_points(s2.p1, s2.p2);
        return intersection(l2);
    }

    Point2d<T> intersection(Line<T> l2) {
        auto l1 = Line<T>::from_points(p1,p2);
        Point2d<T> p = l1.intersection(l2);
        if(is_between<T>(p.x, p1.x, p2.x)) {
            return p;
        } else {
            return {NAN, NAN};
        }
    }
    std::string to_string() {
        stringstream ss;
        ss << "(" << p1.x << "," << p1.y<< ")-(" << p2.x << "," << p2.y << ")";
        return ss.str(); 
    }
};

template<class T=double>
T sign_of(T v) {
    if(v >= 0.){
         return 1.;
    }
    return -1.;
}

template<class T=double>
T fake_laser_reading(const Pose<T> & pose, T theta, const vector<LineSegment<T>> & world) {
    T dx = cos(theta + pose.get_theta());
    T dy = sin(theta + pose.get_theta());
    auto l = Line<T>::from_points({pose.get_x(),pose.get_y()},{pose.get_x()+dx,pose.get_y()+dy});
    T best_d = NAN;
    for( auto segment : world) {
        Point2d<T> p = segment.intersection(l);
        if(isnan(p.x)) {
            continue;
        }
        p.x -= pose.get_x();
        p.y -= pose.get_y();
        if((sign_of(p.x)==sign_of(dx)) && (sign_of(p.y)==sign_of(dy))) {
            T d = p.norm();
            if(isnan(best_d)) {
                best_d = d;
            } else {
                best_d = std::min<T>(d, best_d);
            }
        }
    }

    //std::normal_distribution<T> random_scale(1.0, 0.01);
    return best_d;// * random_scale(random_engine);
}

template <class T=double>
vector<ScanLine<T>> scan_with_twist(vector<LineSegment<T>> & world, int scan_count, T twist_x, T twist_y, T twist_theta, Pose<T> initial_pose = Pose<T>()) {
    vector<ScanLine<T>> output;
    Pose<T> pose = initial_pose;
    for(int i=0; i < scan_count; ++i) {
        T scanner_theta = 2*EIGEN_PI * i / scan_count;;
        T d = fake_laser_reading<>(pose, scanner_theta, world);
        output.emplace_back(scanner_theta, d);
        pose.move({twist_x/scan_count, twist_y/scan_count}, twist_theta/scan_count);
    }
    return output;
}


void print_world(vector<LineSegment<double>> & world) {
    for(auto segment : world) {
        std::cout << segment.to_string() << endl;
    }
}


Stopwatch untwist_timer;
Stopwatch move_scan_timer;
Stopwatch scan_difference_timer;
Stopwatch match_scans_timer;

template <class T=double>
vector<Point2d<T>> untwist_scan(
        vector<Point2d<T>> &twisted_readings, 
        T twist_x, 
        T twist_y, 
        T twist_theta, 
        bool scan_rotation_reversed,
        Pose<T> initial_pose = Pose<T>()) {
    untwist_timer.start();
    int count = twisted_readings.size();
    Pose<T> pose = initial_pose;
    vector<Point2d<T>> untwisted;

    bool ccw = !scan_rotation_reversed;

    if(ccw) {
        // ccw
        for(size_t i = 0; i < twisted_readings.size()+1; ++i) {
            auto p1 = twisted_readings[i%count];
            if(!isnan(p1.x)) {
                auto p2 = pose.Pose2World(p1);
                untwisted.emplace_back(p2);
            }
            pose.move({twist_x/count, twist_y/count}, twist_theta/count);
        }
    } else {
        // cw
        for(int i = twisted_readings.size(); i >= 0; --i) {
            auto p1 = twisted_readings[i%count];
            if(!isnan(p1.x)) {
                auto p2 = pose.Pose2World(p1);
                untwisted.emplace_back(p2);
            }
            pose.move({twist_x/count, twist_y/count}, twist_theta/count);
        }
        std::reverse(untwisted.begin(), untwisted.end());
    }

    untwist_timer.stop();

    return untwisted;
}

template <class T = double>
vector<LineSegment<T>> get_world() {
    vector<LineSegment<T>> world;
    world.emplace_back(-1, 2, 1, 2);
    world.emplace_back(1, 2, 1, 1);
    world.emplace_back(-10, 3, 10, 3);
    world.emplace_back(-10, -3, 10, -3);
    world.emplace_back(10, 2, 10, -3);
    // world.emplace_back(-10, 3, -10, -3);
    return world;
}


template <class T>
struct MinimizeResult{
    vector<T> p;
    T error;
};

template <class T>
inline T abs_sum(vector<T> & v) {
    T rv = 0;
    for(auto p : v) {
        rv += fabs(p);
    }
    return rv;
}

// based loosely on https://martin-thoma.com/twiddle/
template <class T>
MinimizeResult<T> minimize(vector<T> guess, std::function<T(const vector<T>&)> f, T threshold = 0.0003, bool log_goal_steps = false) {
    
    // initialize parameters to guess
    vector<T> p = guess;
    T best_error = f(p);
    float min_dp = 0.00003;

    // potential changes
    auto dp = std::vector<T>(guess.size(), 0.1);
    const T growth_rate = 1.5;
    const auto p_size = p.size();


    while(abs_sum(dp) > threshold) {
        // if(log_goal_steps) {
        //     cout << "looping p "<< p[0] << " " << p[1] << " " << p[2] << " dp:" << dp[0] << " " << dp[1] << " " << dp[2] << endl;
        // }
        for(size_t i = 0; i< p_size; ++i) {
            p[i] += dp[i];
            T error = f(p);
            // if(log_goal_steps) {
            //     cout << "error: " << error << endl;
            // }

            if (error < best_error) {
                best_error = error;
                dp[i] *= growth_rate;
            } else {
                // There was no improvement
                p[i] -= 2*dp[i];  // Go into the other direction
                error = f(p);

                if (error < best_error) {
                  // There was an improvement
                    best_error = error;
                    dp[i] *= growth_rate;
                } else  {
                    // There was no improvement
                    p[i] += dp[i];
                    // As there was no improvement, the step size in either
                    // direction, the step size might simply be too big.
                    dp[i] /= growth_rate;
                }
            }

            // avoid vanishing gradient / delta
            if(dp[i] < min_dp) {
                dp[i] = min_dp;
            }
        }
    }
    MinimizeResult<T> rv;
    rv.p = p;
    rv.error = best_error;
    return rv;
}


template <class T = double>
void move_scan(
    const vector<Point2d<T>> &scan_xy,  
    Pose<T> pose, 
    vector<Point2d<T>> &moved_scan) {
        
    move_scan_timer.start();
    Point2d<T> p, p_new;

    moved_scan.resize(0);

    for(auto & xy : scan_xy) {
        if(!isnan(xy.x)) {
            p.x = xy.x;
            p.y = xy.y;
            p_new = pose.Pose2World(p);
        } else {
            p_new.x = NAN;
            p_new.y = NAN;
        }
        moved_scan.emplace_back(p_new);
    }
    move_scan_timer.stop();
}

template <class T>
T prorate(T x, T x1, T x2, T y1, T y2) {
    return (x-x1) / (x2-x1) * (y2-y1) + y1;
}

template <class T>
inline bool is_ccw(const Point2d<T> & p2, const Point2d<T> & p1) {
    // returns true if p2 is ccw of p1

    // it is ccw if the cross product is positive
    // here we only need to calculate the z term of the cross product
    return (p1.x*p2.y-p1.y*p2.x) >= 0;
}

// use scan2 for "walls"
template <class T>
struct Wall {
    T theta_min;
    T theta_max;
    Point2d<T> p1;
    Point2d<T> p2;
    bool operator < (const Wall & rhs) {
        return this->theta_min < rhs.theta_min;
    }
};

template <class T>
vector<Wall<T>> get_sorted_walls(const vector<Point2d<T>> & scan) {
    vector<Wall<T>> walls;
    walls.reserve(scan.size()-1);
    T theta2 = atan2(scan[0].y,scan[0].x);
    for(int i=0; i < scan.size()-1; ++i) {
        auto & p1 = scan[i];
        auto & p2 = scan[i+1];
        T theta1 = theta2;
        T theta2 = atan2(p2.y,p2.x);

        walls[i] = (theta1 <= theta2) ? Wall<T>{theta1, theta2, p1, p2} : Wall<T>{theta2, theta1, p2, p1};
    }

    std::sort(walls.begin(), walls.end());
    return walls;
}


// computes scan difference of scans without requiring matching equally spaced scan angles
// tbd whether there is a requirement for increasing scan angles
template<class T>
T scan_difference(const vector<Point2d<T>> & scan1, const vector<Point2d<T>> & scan2) {
    
    scan_difference_timer.start();
    ++g_scan_difference_count;

    // walk around scan1, finding correspondences in scan2
    T total_difference = 0;
    int points_compared = 0;

    // index of rays in scan2 to compare

    // i2a,i2b should point to first non-null wall
    size_t i2a = 0;
    size_t i2b = 1;
    while(isnan(scan2[i2a].x) || isnan(scan2[i2b].x) && (i2b < scan2.size())) {
        ++i2a;
        ++i2b;
    }

    int i1 = 0;

    while( isnan(scan1[i1].x) && i1 < scan1.size() ) {
        ++i1;
    }

    auto p1 = scan1[i1];
    auto p2a = scan2[i2a];
    auto p2b = scan2[i2b];
    if(isnan(p1.x) || isnan(p2a.x) || isnan(p2b.x) ) {
        return 0;
    }

    while(true) {

        // if p1 is past wall, grab more wall
        if(is_ccw(p1, p2b)) {
            do {
                ++i2b;
                ++i2a;
            } while (i2b<scan2.size() && (isnan(scan2[i2b].x) || isnan(scan2[i2a].x)) );
    
            if(i2b>=scan2.size()) break; // done if there is no more wall
            p2b = scan2[i2b];
            p2a = scan2[i2a];


            continue;
        }

        // if wall is past p1, grab more points
        if(is_ccw(p2a, p1)) {
            do {
                ++i1;
            } while (isnan(scan1[i1].x) && i1<scan1.size());

            if(i1 >= scan1.size()) break; // done if there are no more points
            p1=scan1[i1];

            continue;
        }

        // if we're here, point must be in wall

        // get distance
        Line<T> l1 = Line<T>::from_points({0,0},p1);
        Line<T> l2 = Line<T>::from_points(scan2[i2a], scan2[i2b]);
        Point2d<T> p = l1.intersection(l2);

        T dx = p.x-p1.x;
        T dy = p.y-p1.y;

        // tbd: what is the best difference metric?
        // total_difference += sqrt(dx*dx+dy*dy);
        //total_difference += fabs(dx)+fabs(dy);
        total_difference -= exp(-1000*(dx*dx+dy*dy));
        ++points_compared;

        // grab next non-null p1 and continue
        do {
            ++i1;
        } while( isnan(scan1[i1].x) && i1 < scan1.size() );

        if(i1 >= scan1.size()) break; // done if there are no more points
        p1 = scan1[i1];
        continue;

    }
    scan_difference_timer.stop();
    return total_difference / points_compared;
}

template <class T> void  get_scan_xy(const vector<ScanLine<T>> & scan, vector<Point2d<T>> & scan_xy) {
    scan_xy.resize(scan.size());
    uint32_t i = 0;
    for(auto & scan_line : scan) {
        scan_xy[i] = Point2d<T>{scan_line.d * cos(scan_line.theta),scan_line.d * sin(scan_line.theta)};
        ++i;
    }
}

template <class T> vector<Point2d<T>> get_scan_xy(const vector<ScanLine<T>> & scan) {
    vector<Point2d<T>> scan_xy;
    get_scan_xy(scan, scan_xy);
    return scan_xy;
}

template <class T = double>
struct ScanMatch {
    Pose<T> delta;
    T score;
};


template <class T = double>
ScanMatch<T> match_scans(const vector<Point2d<T>> & scan1_xy, const vector<Point2d<T>> & scan2_xy, Pose<T> guess, bool log_goal_steps = false) {
    match_scans_timer.start();
    vector<Point2d<T>> scan2b;
    scan2b.reserve(scan2_xy.size());
    auto error_function = [&scan1_xy, &scan2_xy, &scan2b, &log_goal_steps](const vector<T> & params){
        Pose<T> pose(params[0], params[1], params[2]);
        move_scan(scan2_xy, pose, scan2b);
        T d = scan_difference(scan1_xy, scan2b);
        if(log_goal_steps) {
            cout << "difference: " << d << " pose: " << to_string(pose) << endl;
        }

        return d;
    };

    MinimizeResult<T> r = minimize<T>({guess.get_x(),guess.get_y(),guess.get_theta()}, error_function, 0.001, log_goal_steps);
    Pose<T> match(r.p[0], r.p[1], r.p[2]);
    match_scans_timer.stop();
    return {match, r.error};
}


template <class T = double>
ScanMatch<T> match_scans(const vector<ScanLine<T>> & scan1, const vector<ScanLine<T>> & scan2, Pose<T>& guess, bool log_goal_steps) {
    match_scans_timer.start();
    auto scan1_xy = get_scan_xy(scan1);
    auto scan2_xy = get_scan_xy(scan2);
    Pose<T> null_pose;
    match_scans_timer.stop();
    return match_scans(scan1_xy, scan2_xy, guess, log_goal_steps);
}


void test_prorate() {
    double y = prorate(15,10,20,10,40);
    cout << "prorate result: " << y << endl;
}


void test_minimize() {
    auto lambda = [](std::vector<double> v)->double{return fabs(v[0]-3)+fabs(v[1]-5) + fabs(v[2]);};
    auto rv = minimize<double>({0,0,0}, lambda, 1E-10);
    cout << rv.p[0] << ", " << rv.p[1] << ", " << rv.p[2] << " error: " << rv.error << endl;
        
}

template <class T>
void test_move_scan(bool trace = false) {
    auto world = get_world<T>();
    Pose<T> pose2(0, 5, degrees2radians(3));
    auto scan1 = scan_with_twist<T>(world, 360);
    auto scan1_xy = get_scan_xy(scan1);
    vector<Point2d<T>> scan2;
    move_scan(scan1_xy, pose2, scan2);
    if(trace) {
        cout << "original scan, moved scan" << endl;
        for(unsigned i = 0; i < scan2.size(); ++i) {
            cout << " -> "  <<scan1_xy[i].x <<  ", " << scan1_xy[i].y
                 << " -> "  <<scan2[i].x <<  ", " << scan2[i].y  << endl;
                //<< radians2degrees(scan2[i].theta) << "°, " << scan2[i].d << endl;
        }
    }
}



void test_fake_scan() {
    vector<ScanLine<double>> scan;
    vector<LineSegment<double>> world;
    world.emplace_back(-10, 2, 10, 2);
    world.emplace_back(-10, -2, 10,-2);
    Pose<double> pose;
    for( int i = 0; i < 360; ++i) {
        double theta = degrees2radians(i);
        double d = fake_laser_reading<double>(pose, theta, world);
        scan.emplace_back(theta, d);
    }
    print_scan(scan);
}

void test_scan_with_twist() {
    auto world = get_world();
    double twist_theta = degrees2radians(5);
    double twist_x = .2;
    double twist_y = .1;
    double reading_count = 360;
    Pose<double> pose1;
    auto twisted = scan_with_twist(world, reading_count, twist_x, twist_y, twist_theta, pose1);
    auto twisted_xy = get_scan_xy(twisted);
    auto untwisted = untwist_scan(twisted_xy, twist_x, twist_y, twist_theta, false);
    cout << endl << endl << "twisted scan" << endl;
    print_scan(twisted);
    cout << endl << endl << "untwisted scan" << endl;
    //print_scan(untwisted);
}

void test_intersection() {
    double dx = -2.89707;
    double dy = -3;
    Eigen::Vector2d v{dx,dy};
    v = v/v.norm();
    auto s = LineSegment<>({-2.89755,-3},{-2.89707,-3});
    auto l = Line<double>::from_points({0,0},{v(0), v(1)});
    auto p = s.intersection(l);
    cout << "intersection at:" << p.x << ", " << p.y << endl;
    if((sign_of(p.x)==sign_of(dx)) && (sign_of(p.y)==sign_of(dy))) {
    
    cout << "signs ok";
    }
}
