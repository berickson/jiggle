
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>

#include <iomanip>

#include "lidar_mapper.h"

using namespace std;


int min_max_rand(int min, int max) {
  return min + rand() % (max-min) ;
}

int main(int argc, char ** argv) {

    LidarMapper mapper;
    rosbag::Bag bag;
    string bag_path = (argc >= 2) ? argv[1]: "/home/brian/lidar_ws/around-bar-3-x-2020-06-04-16-46-44.bag";
    mapper.bag_path = bag_path; // for logging
    uint32_t scan_count_limit = (argc >= 3) ? strtoul(argv[2],NULL,10): numeric_limits<uint32_t>::max();

    bag.open(bag_path);  // BagMode is Read by default

    rosbag::Bag out_bag;
    out_bag.open("out.bag", rosbag::bagmode::Write);


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

    geometry_msgs::PoseArray pose_array;


    
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
      

      // publish pose_array 
      {
        pose_array.header = scan->header;
        ros::Time time(scan->header.stamp);

        // populate the pose graph
        auto & v = mapper.pose_graph.m_vertices;
        auto & poses = pose_array.poses;
        poses.resize(v.size());
        tf2::Quaternion quaternion;
        for(size_t i = 0; i < v.size(); ++i) {
          auto & pose = v[i].m_property.pose;
          poses[i].position.x = pose.get_x();
          poses[i].position.y = pose.get_y();
          poses[i].position.z = 0;
          quaternion.setRPY(0,0,pose.get_theta());
          poses[i].orientation.x = quaternion.x();
          poses[i].orientation.y = quaternion.y();
          poses[i].orientation.z = quaternion.z();
          poses[i].orientation.w = quaternion.w();
        }

        out_bag.write("/calculated_path",  time, pose_array);
      }

      // publish untwisted as a point cloud
      {
        sensor_msgs::PointCloud point_cloud;
        ros::Time time(scan->header.stamp);
        point_cloud.channels.resize(0);
        point_cloud.header = scan->header;
        auto & untwisted_scan = mapper.pose_graph.m_vertices[mapper.pose_graph.m_vertices.size()-1].m_property.untwisted_scan;
        point_cloud.points.resize(untwisted_scan.size());
        for(uint32_t i = 0; i < untwisted_scan.size(); ++i) {
          point_cloud.points[i].x = untwisted_scan[i].x;
          point_cloud.points[i].y = untwisted_scan[i].y;
          point_cloud.points[i].z = 0;
        }
        out_bag.write("/untwisted_scan",  time, point_cloud);
      }

    }

  
    cerr << "untwist: " << untwist_timer.get_elapsed_seconds() << endl;
    cerr << "move_scan: "  << move_scan_timer.get_elapsed_seconds() << endl;
    cerr << "scan_difference: " <<  scan_difference_timer.get_elapsed_seconds() << endl;
    cerr << "match_scans: " <<  match_scans_timer.get_elapsed_seconds() << endl;
    cerr << "total difference count: " << g_scan_difference_count << endl;
    cerr << "total wrap count: " << g_wrap_count << endl;

    bag.close();

    // Try manually adding loop from scan at index 175 to 112 (around bar dataset)
    if(1)
    {
      cerr << "calculating closures" << endl;
      vector<pair<size_t,size_t>> closures;
    // dx 0.0752851	dy -0.0476339	 dtheta -6.41287
      //for(auto closure : vector<pair<size_t,size_t>> {{175,112}, {159,0}}) {
      //  auto index1 = closure.first;
      //  auto index2 = closure.second;
      
      for(int i = 0; i < 500; ++i) {
        size_t index1 = rand() % mapper.pose_graph.m_vertices.size();
        size_t index2 = rand() % mapper.pose_graph.m_vertices.size();
        //size_t index1 = min_max_rand(mapper.pose_graph.m_vertices.size() * 2 / 3,mapper.pose_graph.m_vertices.size());
        //size_t index2 = min_max_rand(0,mapper.pose_graph.m_vertices.size() / 3);

        if(index1 == index2) continue;


        auto node1 = mapper.pose_graph.m_vertices[index1].m_property;
        auto node2 = mapper.pose_graph.m_vertices[index2].m_property;
        auto m = match_scans(node1.untwisted_scan, node2.untwisted_scan,node1.pose.relative_pose_to(node2.pose));
        cerr << index1 << ", " << index2 << " score" << m.score << endl;
        if(m.score < -100) {
          cerr << "adding edge from " << index1 << " to " << index2 << " with score " << m.score << endl;
          auto e = boost::add_edge(index1, index2, m, mapper.pose_graph);
        }
      }

      // jiggle the graph
      //cerr << "jiggling the loops" << endl;
      //mapper.jiggle();

      //return 0;

    
      
    }
    mapper.write_g2o();
    return 0;


    if(0) {
      mapper.write_path_csv(cout);
      return 0;
    }


    /*
    Test 1: Pick a 5 equally spaced scans to test with, 
    for each one, compare with all other scans, 
    note the match scores, and figure out if there
    is an accuracy pattern, use this to determine a
    good cut-off score for matches.    
    */


   cout << "percent,score,index1,index2,seq1,seq2,d,dx,dy,dtheta" << endl;
   for(float percent: {0.0, 0.2, 0.4, 0.6, 0.8}) {
     size_t index1 = mapper.pose_graph.m_vertices.size() * percent;
     auto node1 = mapper.pose_graph.m_vertices[index1].m_property;
     
     //auto node1=mapper.pose_graph[node1_id];
     for(size_t index2 = 0; index2 < mapper.pose_graph.m_vertices.size(); ++index2) {
       auto node2 = mapper.pose_graph.m_vertices[index2].m_property;
       auto m = match_scans(node1.untwisted_scan, node2.untwisted_scan, node1.pose.relative_pose_to(node2.pose));
       auto & delta = m.delta;
       float dx = delta.get_x();
       float dy = delta.get_y();
       float d = sqrt(dx*dx+dy*dy);
       cout << percent << "," << m.score << "," << index1 << "," << index2 << "," << node1.header.seq << "," << node2.header.seq << "," <<  d << "," <<  dx << "," << dy << ", " << delta.get_theta() << endl;
     }
   }
    


    return 0;

}