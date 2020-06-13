#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <iomanip>

#include "lidar_mapper.h"

using namespace std;



int main(int argc, char ** argv) {

    LidarMapper mapper;
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

  
    cerr << "untwist: " << untwist_timer.get_elapsed_seconds() << endl;
    cerr << "move_scan: "  << move_scan_timer.get_elapsed_seconds() << endl;
    cerr << "scan_difference: " <<  scan_difference_timer.get_elapsed_seconds() << endl;
    cerr << "match_scans: " <<  match_scans_timer.get_elapsed_seconds() << endl;
    cerr << "total difference count: " << g_scan_difference_count << endl;
    cerr << "total wrap count: " << g_wrap_count << endl;

    bag.close();

    mapper.write_path_csv(cout);
    return 0;


    /*
    Test 1: Pick a 5 equally spaced scans to test with, 
    for each one, compare with all other scans, 
    note the match scores, and figure out if there
    is an accuracy pattern, use this to determine a
    good cut-off score for matches.    
    */


   cout << "percent,score,seq1,seq2,dx,dy,dtheta" << endl;
   for(float percent: {0.0, 0.2, 0.4, 0.6, 0.8}) {
     auto node1 = mapper.pose_graph.m_vertices[mapper.pose_graph.m_vertices.size() * percent].m_property;
     
     //auto node1=mapper.pose_graph[node1_id];
     for(auto v2 : mapper.pose_graph.m_vertices) {
       auto node2 = v2.m_property;
       auto m = match_scans(node1.untwisted_scan, node2.untwisted_scan, node1.pose.relative_pose_to(node2.pose));
       auto & delta = m.delta;
       cout << percent << "," << m.score << "," << node1.header.seq << "," << node2.header.seq << "," <<  delta.get_x() << "," << delta.get_y() << ", " << delta.get_theta() << endl;
     }
   }
    


    return 0;

}