#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>


int main() {
    rosbag::Bag bag;
    bag.open("/home/brian/lidar_ws/Team_Hector_MappingBox_Dagstuhl_Neubau.bag");  // BagMode is Read by default

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      sensor_msgs::LaserScan::ConstPtr i = m.instantiate<sensor_msgs::LaserScan>();
      if (i != nullptr)
        std::cout << i->header.stamp << std::endl;
    }

    bag.close();

    std::cout << "done reading bag" << std::endl;

    return 0;

}