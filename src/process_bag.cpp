#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>

#include <iomanip>

using namespace std;

#include "dewarp.cpp"

// converts ros message to format compatible with dewarp
void ros_scan_to_scan_lines(const sensor_msgs::LaserScan & scan, vector<ScanLine<float>> & lines) {
  float angle = scan.angle_min;
  lines.clear();
  lines.resize(scan.ranges.size());
  for(int i = 0; i < scan.ranges.size(); ++i) {
    lines.emplace_back(ScanLine<float>(angle, scan.ranges[i]));
    angle += scan.angle_increment;
  }
}

int main(int argc, char ** argv) {
    rosbag::Bag bag;
    string bag_path = (argc == 2) ? argv[1]: "/home/brian/lidar_ws/Team_Hector_MappingBox_Dagstuhl_Neubau.bag";
    cout << "dx,dy,dtheta,x,y,theta," << bag_path << endl;
    bag.open(bag_path);  // BagMode is Read by default

    uint32_t n_msg = 0, n_scan = 0;

    std::vector<std::string> topics;
    topics.push_back(std::string("/scan"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    vector<ScanLine<float>> lines;
    vector<ScanLine<float>> last_lines;
    Pose<float> pose;

    for(rosbag::MessageInstance const m: view) {
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
      if(true || n_msg%20 == 0) {
        last_lines = lines;
        ros_scan_to_scan_lines(*scan, lines);

        if(n_msg > 2) {
            auto matched_pose = match_scans(lines, last_lines);
            // cout << " -> matched pose: " << to_string(matched_pose) << endl;
            pose.move({matched_pose.get_x(), matched_pose.get_y()}, matched_pose.get_theta());
            cout << matched_pose.get_x() << ", " << matched_pose.get_y() << ", " << matched_pose.get_theta() << ", " << pose.get_x() << ", " << pose.get_y() << ", " << pose.get_theta() << endl;

        }
      }

      ++n_msg;
      if (scan == nullptr) {
        continue;
      }
      ++n_scan;

      if(false && n_scan % 100 == 0) {
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