#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_deserializer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include "rosbag2_cpp/typesupport_helpers.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <iomanip>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_mapper.h"
#include "point_cloud_helper.h"

using namespace std;

int main(int argc, char** argv) {
  LidarMapper mapper;
  rosbag2_cpp::Reader in_bag;
  string bag_path =
      (argc >= 2)
          ? argv[1]
          : "/home/brian/jiggle_ws/data/a3-lab-bar-kitchen-2020-07-14-21-38-09";
  mapper.bag_path = bag_path;  // for logging
  uint32_t scan_count_limit = (argc >= 3) ? strtoul(argv[2], NULL, 10)
                                          : numeric_limits<uint32_t>::max();

  in_bag.open(bag_path);  // BagMode is Read by default

  rosbag2_cpp::Writer out_bag;
  out_bag.open(bag_path + ".out.bag");

  uint32_t n_scan = 0;

  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(std::string("/scan"));
  filter.topics.push_back(std::string("/poseupdate"));
  filter.topics.push_back(std::string("/tf"));

  in_bag.set_filter(filter);

  Pose<float> pose;
  Pose<float> matched_pose(0, 0, 0);

  nav_msgs::msg::Odometry odom;
  bool have_odom = false;

  // geometry_msgs::msg::Transform base_link;

  geometry_msgs::msg::PoseArray pose_array;

  // prepare ros2 bag reader / writer
  auto ros_message =
      std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
  ros_message->time_stamp = 0;
  ros_message->message = nullptr;
  ros_message->allocator = rcutils_get_default_allocator();

  rosbag2_cpp::SerializationFormatConverterFactory factory;
  std::unique_ptr<
      rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
      cdr_deserializer_;

  auto library = rosbag2_cpp::get_typesupport_library(
      "sensor_msgs/msg/LaserScan", "rosidl_typesupport_cpp");
  auto type_support = rosbag2_cpp::get_typesupport_handle(
      "sensor_msgs/msg/LaserScan", "rosidl_typesupport_cpp", library);
  cdr_deserializer_ = factory.load_deserializer("cdr");

  while (in_bag.has_next()) {
    // Read scan_msg from bag
    sensor_msgs::msg::LaserScan scan_msg;
    {
      // scan_msg = in_bag.read_next<sensor_msgs::msg::LaserScan>();

      auto m = in_bag.read_next();

      rclcpp::SerializedMessage extracted_serialized_msg(*m->serialized_data);

      auto topic = m->topic_name;
      if (topic != "/scan") {
        // cout << "skipping message from " << topic << endl;
        continue;
      }

      ros_message->message = &scan_msg;

      cdr_deserializer_->deserialize(m, type_support, ros_message);

      cout << topic << "frame_id: " << scan_msg.header.frame_id
           << " sec: " << scan_msg.header.stamp.sec
           << " nanosec: " << scan_msg.header.stamp.nanosec
           << " scan_time: " << scan_msg.scan_time
           << " reading count:" << scan_msg.ranges.size() << endl;

      ++n_scan;
    }

    // use the scan time for all messages
    rclcpp::Time time(scan_msg.header.stamp);

    // incorporate scan_msg into map
    {
      static uint32_t last_closure_scan = 0;
      if (n_scan > scan_count_limit) break;
      bool processed = mapper.add_scan(scan_msg);
      if(!processed) continue;
      bool enable_closure = false;
      if (enable_closure && n_scan > 2 && n_scan - last_closure_scan > 20) {
        mapper.do_loop_closure();
        last_closure_scan = n_scan;
      }
    }

    // publish pose_array
    {
      pose_array.header = scan_msg.header;
      pose_array.header.frame_id = "map";

      // populate the pose graph
      auto& v = mapper.pose_graph.m_vertices;
      auto& poses = pose_array.poses;
      poses.resize(v.size());
      tf2::Quaternion quaternion;
      for (size_t i = 0; i < v.size(); ++i) {
        auto& pose = v[i].m_property.pose;
        poses[i].position.x = pose.get_x();
        poses[i].position.y = pose.get_y();
        poses[i].position.z = 0;
        quaternion.setRPY(0, 0, pose.get_theta());
        poses[i].orientation.x = quaternion.x();
        poses[i].orientation.y = quaternion.y();
        poses[i].orientation.z = quaternion.z();
        poses[i].orientation.w = quaternion.w();
      }

      out_bag.write<geometry_msgs::msg::PoseArray>(pose_array,
                                                   "/calculated_path", time);
    }

    // publish tf (map to neato_laser)
    {
      Pose<float>& pose = mapper.pose;
      geometry_msgs::msg::TransformStamped ts;
      ts.header = scan_msg.header;
      ts.header.frame_id = "map";
      ts.child_frame_id = "laser";
      ts.transform.translation.x = pose.get_x();
      ts.transform.translation.y = pose.get_y();
      ts.transform.translation.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, pose.get_theta());
      ts.transform.rotation.x = q.x();
      ts.transform.rotation.y = q.y();
      ts.transform.rotation.z = q.z();
      ts.transform.rotation.w = q.w();

      tf2_msgs::msg::TFMessage tf_msg;
      tf_msg.transforms.push_back(ts);

      out_bag.write(tf_msg, "/tf", time);
    }

    // publish pose graph as makers
    {
      visualization_msgs::msg::Marker closure_lines;
      closure_lines.header.frame_id = "map";
      closure_lines.header.stamp = scan_msg.header.stamp;
      closure_lines.type = visualization_msgs::msg::Marker::LINE_LIST;

      closure_lines.ns = "pose_graph";
      closure_lines.id = 0;
      visualization_msgs::msg::MarkerArray markers;
      closure_lines.action = visualization_msgs::msg::Marker::ADD;

      closure_lines.pose.position.x = 0.0;
      closure_lines.pose.position.y = 0.0;
      closure_lines.pose.position.z = 0.0;
      closure_lines.pose.orientation.x = 0.0;
      closure_lines.pose.orientation.y = 0.0;
      closure_lines.pose.orientation.z = 0.0;
      closure_lines.pose.orientation.w = 1.0;
      closure_lines.scale.x = 0.02;
      closure_lines.color.r = 0.0;
      closure_lines.color.g = 1.0;
      closure_lines.color.b = 1.0;
      closure_lines.color.a = 1.0;
      closure_lines.frame_locked = true;

      visualization_msgs::msg::Marker path_lines;
      path_lines.header.frame_id = "map";
      path_lines.header.stamp = scan_msg.header.stamp;
      path_lines.type = visualization_msgs::msg::Marker::LINE_LIST;

      path_lines.ns = "pose_graph";
      path_lines.id = 1;
      path_lines.action = visualization_msgs::msg::Marker::ADD;

      path_lines.pose.position.x = 0.0;
      path_lines.pose.position.y = 0.0;
      path_lines.pose.position.z = 0.0;
      path_lines.pose.orientation.x = 0.0;
      path_lines.pose.orientation.y = 0.0;
      path_lines.pose.orientation.z = 0.0;
      path_lines.pose.orientation.w = 1.0;
      path_lines.scale.x = 0.02;
      path_lines.color.r = 0.6;
      path_lines.color.g = 0.3;
      path_lines.color.b = 1.0;
      path_lines.color.a = 1.0;
      path_lines.frame_locked = true;


      visualization_msgs::msg::Marker path_vertices;
      path_vertices.header.frame_id = "map";
      path_vertices.header.stamp = scan_msg.header.stamp;
      path_vertices.type = visualization_msgs::msg::Marker::SPHERE_LIST;

      path_vertices.ns = "pose_graph";
      path_vertices.id = 2;
      path_vertices.action = visualization_msgs::msg::Marker::ADD;

      path_vertices.pose.position.x = 0.0;
      path_vertices.pose.position.y = 0.0;
      path_vertices.pose.position.z = 0.0;
      path_vertices.pose.orientation.x = 0.0;
      path_vertices.pose.orientation.y = 0.0;
      path_vertices.pose.orientation.z = 0.0;
      path_vertices.pose.orientation.w = 1.0;
      path_vertices.scale.x = 0.04;
      path_vertices.scale.y = 0.04;
      path_vertices.scale.z = 0.04;
      path_vertices.color.r = 0.1;
      path_vertices.color.g = 0.1;
      path_vertices.color.b = 0.1;
      path_vertices.color.a = 1.0;
      path_vertices.frame_locked = true;



      /// grab poses from pose graph vertices
      auto& v = mapper.pose_graph.m_vertices;
      auto& edges = mapper.pose_graph.m_edges;
      closure_lines.points.reserve(edges.size());
      for (auto& edge : edges) {
        // The line list needs two points for each line
        geometry_msgs::msg::Point p;

        auto &pose1 = v[edge.m_source].m_property.pose;
        geometry_msgs::msg::Point p1;
        p1.x = pose1.get_x();
        p1.y = pose1.get_y();
        p1.z = 0;

        auto &pose2 = v[edge.m_target].m_property.pose;
        geometry_msgs::msg::Point p2;
        p2.x = pose2.get_x();
        p2.y = pose2.get_y();
        p2.z = 0;

        if(abs((int)edge.m_source-(int)edge.m_target)!=1) {
          closure_lines.points.push_back(p1);
          closure_lines.points.push_back(p2);
        } else {
          path_lines.points.push_back(p1);
          path_lines.points.push_back(p2);
        }
      }

      for(auto & vertice : mapper.pose_graph.m_vertices) {
        geometry_msgs::msg::Point p;
        p.x = vertice.m_property.pose.get_x();
        p.y = vertice.m_property.pose.get_y();
        path_vertices.points.push_back(p);
      }

      if(path_lines.points.size() > 0) {
        out_bag.write(path_lines, "/pose_graph_path", time);
      }
      if(closure_lines.points.size() > 0) {
        out_bag.write(closure_lines, "/pose_graph_closures", time);
      }
      if(path_vertices.points.size() > 0) {
        out_bag.write(path_vertices, "/pose_graph_vertices", time);
      }


    }

    // publish path as markers
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = scan_msg.header.stamp;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

      marker.ns = "robot_path";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.025;
      marker.color.r = 0.7;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.frame_locked = true;

      /// grab poses from pose graph vertices
      auto& v = mapper.pose_graph.m_vertices;
      marker.points.reserve(v.size());
      for (int i = 0; i < v.size(); ++i) {
        auto& pose = v[i].m_property.pose;
        geometry_msgs::msg::Point p;
        p.x = pose.get_x();
        p.y = pose.get_y();
        p.z = 0;
        marker.points.push_back(p);
      }
      out_bag.write(marker, "/path_marker", time);
    }

    // publish untwisted as a PointCloud2
    {
      sensor_msgs::msg::PointCloud2 point_cloud;
      point_cloud.header = scan_msg.header;
      auto& untwisted_scan =
          mapper.pose_graph.m_vertices[mapper.pose_graph.m_vertices.size() - 1]
              .m_property.untwisted_scan;
      set_point_cloud_points(point_cloud, untwisted_scan);

      out_bag.write(point_cloud, "/untwisted_scan", time);
    }

    // write original scan as PointCloud2 for tool support
    {
      vector<ScanLine<float>> lines;
      ros_scan_to_scan_lines(scan_msg, lines);
      vector<Point2d<float>> scan_xy;
      get_scan_xy<float>(lines, scan_xy);

      sensor_msgs::msg::PointCloud2 point_cloud;
      point_cloud.header = scan_msg.header;
      set_point_cloud_points(point_cloud, scan_xy);

      out_bag.write(point_cloud, "/scan_cloud", time);
    }

    // // write a map by projecting all scans to their poses
    // if (n_scan % 10 == 0) {
    //   vector<Point2d<float>> points;

    //   auto& v = mapper.pose_graph.m_vertices;
    //   for (int i = 0; i < v.size(); ++i) {
    //     auto& node = v[i].m_property;
    //     auto& pose = node.pose;
    //     auto& untwisted = node.untwisted_scan;
    //     for (auto& point : untwisted) {
    //       auto new_point = pose.Pose2World(point);
    //       points.push_back(new_point);
    //     }
    //   }
    //   sensor_msgs::msg::PointCloud2 point_cloud;
    //   point_cloud.header = scan_msg.header;
    //   point_cloud.header.frame_id = "map";
    //   set_point_cloud_points(point_cloud, points);

    //   out_bag.write(point_cloud, "/map_cloud", time);
    // }
  }
  mapper.write_path_csv(std::cout);

  cout << "time untwisting: " << untwist_timer.get_elapsed_seconds() << " count: " << untwist_timer.start_count << endl;
  cout << "time moving: " << move_scan_timer.get_elapsed_seconds() << " count: " << move_scan_timer.start_count << endl;
  cout << "time diffing: " << scan_difference_timer.get_elapsed_seconds() << " count: " << scan_difference_timer.start_count<< endl;
  cout << "time closing loops: " << loop_closure_timer.get_elapsed_seconds() << " count: " << loop_closure_timer.start_count << endl;
  cout << "time adding scans: " << add_scan_timer.get_elapsed_seconds() << " count: " << add_scan_timer.start_count << endl;
  cout << "g2o time: " << g2o_timer.get_elapsed_seconds() << " count: " << g2o_timer.start_count << endl;
  cout << "lidar_odom time: " << lidar_odom_timer.get_elapsed_seconds() << " count: " << lidar_odom_timer.start_count << endl;
  cout << "total time matching: " << match_scans_timer.get_elapsed_seconds() << " count: " << scan_difference_timer.start_count<< endl;
}
