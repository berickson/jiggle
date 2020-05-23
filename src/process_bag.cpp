#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>

#include <iomanip>

using namespace std;

#include "dewarp.cpp"

// converts ros message to format compatible with dewarp
void ros_scan_to_scan_lines(const sensor_msgs::LaserScan & scan, vector<ScanLine<float>> & lines) {
  float angle = scan.angle_min;
  lines.resize(scan.ranges.size());
  for(int i = 0; i < scan.ranges.size(); ++i) {
    float range = scan.ranges[i];
    if(range < scan.range_min || range > scan.range_max) {
      range = NAN;
    }

    lines[i] = ScanLine<float>(angle, range );
    angle += scan.angle_increment;
  }
}

void print_scan(const vector<ScanLine<float>> & lines) {
}

int main(int argc, char ** argv) {
    bool trace_twist = true;
    rosbag::Bag bag;
    string bag_path = (argc == 2) ? argv[1]: "/home/brian/lidar_ws/Team_Hector_MappingBox_L101_Building.bag";
    if(!trace_twist) {
      cout << "sec,scan_sec,frame,scantime,twist_x,twist_y,twist_theta,dx,dy,dtheta,x,y,theta," << bag_path << endl;
    }
    bag.open(bag_path);  // BagMode is Read by default

    uint32_t n_msg = 0, n_scan = 0;

    std::vector<std::string> topics;
    topics.push_back(std::string("/scan"));
    topics.push_back(std::string("/solution"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    vector<ScanLine<float>> lines;
    vector<ScanLine<float>> last_lines;
    Pose<float> pose;
    float last_dx=0;
    float last_dy=0;
    float last_theta = 0;
    
    nav_msgs::Odometry odom;
    bool have_odom = false;

    for(rosbag::MessageInstance const m: view) {
      
      if(m.isType<nav_msgs::Odometry>())
      {
        nav_msgs::Odometry::ConstPtr new_odom = m.instantiate<nav_msgs::Odometry>();
        odom = *new_odom;
        have_odom = true;
        continue;
      }
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();


      if(scan==NULL) continue;

      ros::Time scan_time = m.getTime();
      static ros::Time start_time;
      if(n_msg == 0) {
        start_time = scan_time;
      }
      auto elapsed = scan_time - start_time;

      if(n_msg%1 == 0) {
        last_lines = lines;
        ros_scan_to_scan_lines(*scan, lines);

        if(n_msg > 2) {
            auto & twist = odom.twist.twist;
            auto matched_pose = match_scans(last_lines, lines);
            float scan_factor = 1;
            if(trace_twist && n_scan == 1000) {
              auto untwisted = untwist_scan<float>(lines, matched_pose.get_x()/scan_factor, matched_pose.get_y()/scan_factor, matched_pose.get_theta()/scan_factor);
              cout << "theta, d, x, y, untwisted_theta, untwisted_d, untwisted_x, untwisted_y" << endl;
              for(int i = 0; i < untwisted.size(); ++i ) {
                auto & l = lines[i];
                auto & u = untwisted[i];
                cout << l.theta << ", " << l.d << ", " << l.d*cos(l.theta)  << ", " << l.d*sin(l.theta) << ", " << u.theta << ", " << u.d << ", " << u.d*cos(u.theta)  << ", " << u.d*sin(u.theta) << endl;
              }
              //matched_pose = match_scans(last_lines, lines);
              //cout << " -> matched pose: " << to_string(matched_pose) << endl;
            }
            pose.move({matched_pose.get_x(), matched_pose.get_y()}, matched_pose.get_theta());
            if(!trace_twist) {
              cout << elapsed.toSec()  << ", "
                << (scan->header.stamp-start_time).toSec() << ", "
                << scan->header.frame_id << ", "
                << scan->scan_time << ", " 
                << twist.linear.x << ", "
                << twist.linear.y << ", "
                << twist.angular.z << ", "
                << matched_pose.get_x()  << ", " 
                << matched_pose.get_y()  << ", " 
                << matched_pose.get_theta() << ", " 
                << pose.get_x() << ", " 
                << pose.get_y() << ", "  
                << pose.get_theta() << endl;
            }

            last_dx = matched_pose.get_x();
            last_dy = matched_pose.get_y();
            last_theta = matched_pose.get_theta();
        }
      }

      ++n_msg;
      if (scan == nullptr) {
        continue;
      }
      ++n_scan;

      if(false && n_scan == 100) {
        std::cout << "scan " << n_scan << " " << scan->header.stamp << std::endl;
        for( auto range : scan->ranges ) {
          std::cout << fixed << std::setprecision(3) << range << " ";
        }
      cout << endl;
      }

    }

    bag.close();

    return 0;

}