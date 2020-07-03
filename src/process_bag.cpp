
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


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

        // write again as PointCloud2 as that is what many tools expect
        sensor_msgs::PointCloud2 point_cloud2;
        sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
        out_bag.write("/untwisted_scan2", time, point_cloud2);
      }

        // also write original scan as PointCloud2 for tool support
      {
        ros::Time time(scan->header.stamp);
        // sensor_msgs::PointCloud point_cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;

        pcl_conversions::toPCL(scan->header, cloud_xyz.header);

        vector<ScanLine<float>> lines;
        ros_scan_to_scan_lines(*scan, lines);
        vector<Point2d<float>> scan_xy;
        get_scan_xy<float>(lines, scan_xy);
        
        cloud_xyz.points.resize(scan_xy.size());
        for(uint32_t i = 0; i < scan_xy.size(); ++i) {
          cloud_xyz.points[i].x = scan_xy[i].x;
          cloud_xyz.points[i].y = scan_xy[i].y;
          cloud_xyz.points[i].z  = 0;

        }


        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(cloud_xyz, pcl_pc2);
        sensor_msgs::PointCloud2 point_cloud2;
        //sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
        pcl_conversions::moveFromPCL(pcl_pc2, point_cloud2);
        out_bag.write("/scan2", time, point_cloud2);


      }

    }

  
    cerr << "untwist: " << untwist_timer.get_elapsed_seconds() << endl;
    cerr << "move_scan: "  << move_scan_timer.get_elapsed_seconds() << endl;
    cerr << "scan_difference: " <<  scan_difference_timer.get_elapsed_seconds() << endl;
    cerr << "match_scans: " <<  match_scans_timer.get_elapsed_seconds() << endl;
    cerr << "total difference count: " << g_scan_difference_count << endl;
    cerr << "total wrap count: " << g_wrap_count << endl;

    bag.close();

    // find closures
    if(1)
    {
      cerr << "calculating closures" << endl;
      vector<pair<size_t,size_t>> closures;
      
      for(int i = 0; i < 500; ++i) {
        size_t index1 = rand() % mapper.pose_graph.m_vertices.size();
        size_t index2 = rand() % mapper.pose_graph.m_vertices.size();

        if(labs(index1 - index2) < 30) continue;


        auto node1 = mapper.pose_graph.m_vertices[index1].m_property;
        auto node2 = mapper.pose_graph.m_vertices[index2].m_property;
        // auto m = match_scans(node1.untwisted_scan, node2.untwisted_scan,node1.pose.relative_pose_to(node2.pose));
        const Pose<float> null_pose;
        auto m = match_scans(node1.untwisted_scan, node2.untwisted_scan, null_pose);
        // cerr << index1 << ", " << index2 << " score" << m.score << endl;
        if(m.score < -150 && m.delta.get_polar().r > 0.1 ) {
          cerr << "adding edge from " << index1 << " to " << index2 << " with score " << m.score << " length " << m.delta.get_polar().r << endl;
          auto e = boost::add_edge(index1, index2, m, mapper.pose_graph);
        }
      }

   
      
    }
    mapper.write_g2o("/home/brian/g2o/bin/path.g2o");
    system("/home/brian/g2o/bin/g2o -o /home/brian/g2o/bin/path_out.g2o /home/brian/g2o/bin/path.g2o");
    
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