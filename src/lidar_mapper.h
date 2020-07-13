#include "dewarp.cpp"
#include <vector>

#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

#include <fstream>
#include <string>

inline float distance(float dx, float dy) {
  return sqrt(dx*dx+dy*dy);
}

int min_max_rand(int min, int max) {
  return min + rand() % (max+1-min) ;
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


    if(n_scan % scans_per_match == 0) {
      ros_scan_to_scan_lines(*scan, lines);
      scan_xy = get_scan_xy(lines);

      auto v = boost::add_vertex(pose_graph);

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
          auto e = boost::add_edge(v-1,v, m ,pose_graph);
        
          // nodes.emplace_back(node);
          //pose_graph[pose_graph.m_vertices.size()] = node;
      }
      Node & node = pose_graph[v];
      node.header = scan->header;
      node.pose = pose;
      pose_graph[v] = node;

      last_scan_xy = scan_xy;
      node.untwisted_scan = scan_xy;
    }

    ++n_scan;
  }

  void write_g2o(std::string output_path) {
    
    std::ofstream f (output_path);
    auto & g = pose_graph;
    for(int i = 1; i < num_vertices(g); ++i) {
      for(auto ep = in_edges(i, g);ep.first != ep.second; ++ep.first) {
        const auto & e = *ep.first;
        const ScanMatch<float> & match = g[e];
        const char * info = "1 0 0 1 0 1"; // upper diagonal of 3x3 information matrix 
        
        
    
        
        f << "EDGE_SE2" << " "  <<  e.m_source << " " << e.m_target  << " " <<   match.delta.get_x() << " " <<   match.delta.get_y() << " " <<   match.delta.get_theta() << " " << info<< endl;
        
      }
    }
    f << "FIX 0" << endl;
  }

  void read_g2o(std::string input_path) {
    std::ifstream f (input_path);
    while(!f.eof()) {
      std::string l;
      std::getline(f, l);
      if(l.size() > 0) {
        std::stringstream ss(l);
        std::string token;
        std::getline(ss, token, ' ');
        if(token == "VERTEX_SE2") {
          int n;
          float x,y,theta;
          ss >> n >> x >> y >> theta;
          // cerr << token << " " << n << " " << x << " " << y << " " << theta << endl;
          Node & node = pose_graph[n];
          node.pose = Pose<float>(x,y,theta);
        }
      }
      pose = pose_graph[pose_graph.m_vertices.size()-1].pose;
    }
  }

  void do_loop_closure() {
    bool trace = false;
    // don't re-check nodes
    static uint32_t last_index_checked = 0;

    if(last_index_checked >= pose_graph.m_vertices.size()) return;

    if (trace) cerr << "calculating closures" << endl;
    vector<pair<size_t,size_t>> closures;
    
    uint32_t closure_count = 0;
    for(int i = 0; i < 300; ++i) {
      size_t index1 = min_max_rand(last_index_checked+1,  pose_graph.m_vertices.size()-1);
      size_t index2 = min_max_rand(0, last_index_checked);
      if (trace) cerr << "indexes: " << index1 << ", " << index2 << endl;

      if(labs(index1 - index2) < 30) continue;


      auto node1 = pose_graph.m_vertices[index1].m_property;
      auto node2 = pose_graph.m_vertices[index2].m_property;
      auto starting_diff = node1.pose.relative_pose_to(node2.pose);
      double d = starting_diff.get_polar().r;
      if(d>2) continue;
      const Pose<float> null_pose;
      auto m = match_scans(node1.untwisted_scan, node2.untwisted_scan, null_pose);
      //auto m = match_scans(node1.untwisted_scan, node2.untwisted_scan,node1.pose.relative_pose_to(node2.pose));
      double d_new = m.delta.get_polar().r;
      // cerr << index1 << ", " << index2 << " score" << m.score << endl;
      if(m.score < -190 && d_new > 0.05 && d_new < 1) {
        if(trace) cerr << "adding edge from " << index1 << " to " << index2 << " with score " << m.score <<  " d_new " << d_new << " d " << d
        << " pose " << to_string(starting_diff) << " pose_new " << to_string(m.delta) << endl;
        auto e = boost::add_edge(index1, index2, m, pose_graph);
        ++closure_count;
        break;
      }
    }

    last_index_checked = pose_graph.m_vertices.size()-1;

    if(closure_count > 0) {
      this->write_g2o("/home/brian/g2o/bin/path.g2o");
      system("/home/brian/g2o/bin/g2o -o /home/brian/g2o/bin/path_out.g2o /home/brian/g2o/bin/path.g2o");
      this->read_g2o("/home/brian/g2o/bin/path_out.g2o");
      if(trace) cerr << "done closing" << endl;
    }
  }

  void jiggle() {
    auto & g = pose_graph;
    for(int pass = 0; pass< 100; ++pass) {
        for(int i = 1; i < num_vertices(g); ++i) {
            uint32_t n_estimates = 0;
            double sum_x = 0;
            double sum_y = 0;
            double sum_theta = 0;
            for(auto ep = in_edges(i, g);ep.first != ep.second; ++ep.first) {
                // edge to our edge
                
                Pose<float> & delta = g[*ep.first].delta;
                Polar<float> delta_polar = delta.get_polar();
                Pose<float> & from_pose = g[ep.first->m_source].pose; 
                if(from_pose.get_x() != NAN) {
                    sum_x += from_pose.get_x() + cos(from_pose.get_theta() + delta_polar.theta) * delta_polar.r;
                    sum_y += from_pose.get_y() + sin(from_pose.get_theta() + delta_polar.theta) * delta_polar.r;
                    sum_theta += from_pose.get_theta() + delta.get_theta();
                    ++n_estimates;
                }
            }
            if(n_estimates > 0) {
                // estimate = average calculated pose from connected poses
                // pose is now estimated
                g[i].pose = Pose<float>(sum_x / n_estimates, sum_y / n_estimates, sum_theta / n_estimates);
            }
        }


        // cout << "pass: " << pass << endl;
        // // print_poses(poses);
        // cout << "     x,     y, theta" << endl;
        // for(int i = 0; i < num_vertices(g); ++ i) {
        //     Pose& pose = g[i];
        //     cout << std::setw(6)  << std::setprecision(4) << pose.x << "," << std::setw(6) << std::setprecision(4) << pose.y << "," << std::setw(6) << pose.theta << endl;
        // }

        // cout << endl;

    }

  }

};
