#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

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



class LidarMapper {
public:  

  bool trace_twist = false;
  uint32_t n_scan = 0;
  vector<ScanLine<float>> lines;
  vector<Point2d<float>> scan_xy;
  vector<Point2d<float>> last_scan_xy;
  Pose<float> matched_pose;
  Pose<float> pose;
  string bag_path;

  struct Node {
    Pose<float> pose;
    vector<Point2d<float>> untwisted_scan;
  };

  vector<Node> nodes;


  void add_scan(sensor_msgs::LaserScan::ConstPtr scan) {
    uint32_t scans_per_match = 1;
    ++n_scan;
    if((n_scan-1) % scans_per_match == 0) {
      last_scan_xy = scan_xy;
      ros_scan_to_scan_lines(*scan, lines);
      scan_xy = get_scan_xy(lines);

      if(n_scan==1) {
        cout << "frame,sec,nsec,scan_time,dx,dy,dtheta,x,y,theta,bag_path" << endl;
      } else {
          //auto & twist = odom.twist.twist;
          auto untwisted = untwist_scan<float>(
              scan_xy, 
              matched_pose.get_x()/scans_per_match, 
              matched_pose.get_y()/scans_per_match, 
              matched_pose.get_theta()/scans_per_match);

          matched_pose = match_scans(last_scan_xy, untwisted, matched_pose);
          for(uint32_t i = 0; i < 2; ++i) {
            untwisted = untwist_scan<float>(
              scan_xy, 
              matched_pose.get_x()/scans_per_match, 
              matched_pose.get_y()/scans_per_match, 
              matched_pose.get_theta()/scans_per_match);
            matched_pose = match_scans(last_scan_xy, untwisted, matched_pose);
          }

          scan_xy = untwisted; // save for next time
          pose.move({matched_pose.get_x(), matched_pose.get_y()}, matched_pose.get_theta());
          if(trace_twist) {
            cout 
              << scan->header.seq << ", "
              << scan->header.stamp.sec << ", "
              << scan->header.stamp.nsec << ", "
              << scan->scan_time << ", " 
              << matched_pose.get_x()  << ", " 
              << matched_pose.get_y()  << ", " 
              << matched_pose.get_theta() << ", " 
              << pose.get_x() << ", " 
              << pose.get_y() << ", "  
              << pose.get_theta() << ","
              << bag_path << endl;
          }
          Node node;
          node.pose = pose;
          node.untwisted_scan = scan_xy;
          nodes.emplace_back(node);
      }
    }
  }
};

int main(int argc, char ** argv) {

    LidarMapper mapper;
    mapper.trace_twist = true;
    rosbag::Bag bag;
    string bag_path = (argc >= 2) ? argv[1]: "/home/brian/lidar_ws/around-bar-3-x-2020-06-04-16-46-44.bag";
    mapper.bag_path = bag_path; // for logging
    uint32_t scan_count_limit = (argc >= 3) ? strtoul(argv[2],NULL,10): numeric_limits<uint32_t>::max();

    bag.open(bag_path);  // BagMode is Read by default

    uint32_t n_msg = 0, n_scan = 0;

    std::vector<std::string> topics;
    topics.push_back(std::string("/scan"));
    topics.push_back(std::string("/poseupdate"));
    topics.push_back(std::string("/tf"));
    //topics.push_back(std::string("/solution"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    Pose<float> pose;
    Pose<float> matched_pose(0,0,0);
    
    nav_msgs::Odometry odom;
    bool have_odom = false;

    geometry_msgs::Transform base_link;


    
    for(rosbag::MessageInstance const m: view) {

      ++n_msg;

      if(m.isType<tf::tfMessage>()) {
        tf::tfMessage::ConstPtr new_tf = m.instantiate<tf::tfMessage>();
        if(new_tf->transforms[0].child_frame_id == "/base_footprint") {
          base_link = new_tf->transforms[0].transform;
        }
        continue;
      }

      
      if(m.isType<nav_msgs::Odometry>())
      {
        nav_msgs::Odometry::ConstPtr new_odom = m.instantiate<nav_msgs::Odometry>();
        odom = *new_odom;
        have_odom = true;
        continue;
      }
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();


      if(scan==NULL) continue;
      ++n_scan;
      if(n_scan > scan_count_limit) break;
      mapper.add_scan(scan);
    }

    bag.close();

    cerr << "untwist: " << untwist_timer.get_elapsed_seconds() << endl;
    cerr << "move_scan: "  << move_scan_timer.get_elapsed_seconds() << endl;
    cerr << "scan_difference: " <<  scan_difference_timer.get_elapsed_seconds() << endl;
    cerr << "match_scans: " <<  match_scans_timer.get_elapsed_seconds() << endl;
    cerr << "total difference count: " << g_scan_difference_count << endl;
    cerr << "total wrap count: " << g_wrap_count << endl;


    return 0;

}