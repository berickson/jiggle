#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>

#include <iomanip>

using namespace std;

int main(int argc, char ** argv) {
    rosbag::Bag bag;
    string bag_path = (argc == 2) ? argv[1]: "/home/brian/lidar_ws/Team_Hector_MappingBox_Dagstuhl_Neubau.bag";
    cout << bag_path << endl;
    bag.open(bag_path);  // BagMode is Read by default

    uint32_t n_msg = 0, n_scan = 0;

    std::vector<std::string> topics;
    topics.push_back(std::string("/scan"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m: view) {
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
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

    std::cout << "done reading bag n_msg: " << n_msg << " n_scan:" << n_scan << std::endl;

    return 0;

}