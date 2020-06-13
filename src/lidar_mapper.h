#include "dewarp.cpp"
#include <vector>

#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

inline float distance(float dx, float dy) {
  return sqrt(dx*dx+dy*dy);
}

// converts ros message to format compatible with dewarp
inline void ros_scan_to_scan_lines(const sensor_msgs::LaserScan & scan, vector<ScanLine<float>> & lines) {
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


class LidarMapper {
public:  

  uint32_t n_scan = 0;
  vector<ScanLine<float>> lines;
  vector<Point2d<float>> scan_xy;
  vector<Point2d<float>> last_scan_xy;
  Pose<float> diff;
  Pose<float> pose;
  string bag_path;

  struct Node {
    std_msgs::Header header;
    Pose<float> pose;
    vector<Point2d<float>> untwisted_scan;
  };

  typedef boost::adjacency_list<
      boost::listS, boost::vecS, boost::bidirectionalS,
      Node, ScanMatch<float>>
    PoseGraph;
  PoseGraph pose_graph;


  //vector<Node> nodes;

  void write_path_csv(std::ostream & o) {
    cout << "frame,sec,nsec,score,r,dx,dy,dtheta,x,y,theta,bag_path" << endl;

    for(auto vd : boost::make_iterator_range(vertices(pose_graph)) ){
      auto & node = pose_graph[vd];
      ScanMatch<float> match;
      if(boost::in_degree(vd, pose_graph) > 0) {
        auto range = boost::in_edges(vd, pose_graph);
        match = pose_graph[*range.first];
      }
      o 
        << node.header.seq << ","
        << node.header.stamp.sec << ","
        << node.header.stamp.nsec << ","
        << match.score  << "," 
        << distance(match.delta.get_x(), match.delta.get_y()) << ","
        << match.delta.get_x()  << "," 
        << match.delta.get_y()  << "," 
        << match.delta.get_theta() << "," 
        << node.pose.get_x() << "," 
        << node.pose.get_y() << ","  
        << node.pose.get_theta() << ","
        << bag_path << endl;
    }
  }


  void add_scan(sensor_msgs::LaserScan::ConstPtr scan) {
    uint32_t scans_per_match = 1;
    ros_scan_to_scan_lines(*scan, lines);
    scan_xy = get_scan_xy(lines);

    auto v = boost::add_vertex(pose_graph);
    Node & node = pose_graph[v];
    node.header = scan->header;
    node.pose = pose;
    pose_graph[v] = node;


    if(n_scan % scans_per_match == 0) {

      if(n_scan>0) {
          //auto & twist = odom.twist.twist;
          auto untwisted = untwist_scan<float>(
              scan_xy, 
              diff.get_x()/scans_per_match, 
              diff.get_y()/scans_per_match, 
              diff.get_theta()/scans_per_match);

          auto m = match_scans(last_scan_xy, untwisted, diff);
          diff = m.delta;
          for(uint32_t i = 0; i < 2; ++i) {
            untwisted = untwist_scan<float>(
              scan_xy, 
              diff.get_x()/scans_per_match, 
              diff.get_y()/scans_per_match, 
              diff.get_theta()/scans_per_match);

            m = match_scans(last_scan_xy, untwisted, diff);
            diff = m.delta;
          }

          scan_xy = untwisted; // save for next time
          pose.move({diff.get_x(), diff.get_y()}, diff.get_theta());
          auto e = boost::add_edge(v-scans_per_match,v, m ,pose_graph);
        
          // nodes.emplace_back(node);
          //pose_graph[pose_graph.m_vertices.size()] = node;
      }
      last_scan_xy = scan_xy;

    }
    node.untwisted_scan = scan_xy;

    ++n_scan;
  }
};
