#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <laser_geometry/laser_geometry.hpp> 

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include "sensor_msgs/point_cloud_conversion.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer_interface.h>

#include <vector>

#include "lidar_mapper.h"

using namespace std;

class LidarOdometer : public rclcpp::Node {
public:
    vector<Point2d<float>> scan_xy;
    vector<Point2d<float>> last_scan_xy;
    geometry_msgs::msg::TransformStamped tf_msg;
    tf2::Transform transform;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr untwisted_point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr untwisted_point_cloud2_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

    LidarOdometer() 
    : Node("lidar_odometer")
    {
        transform.setOrigin({0.0,0.0,0.0});
        tf2::Quaternion q;
        q.setRPY(0,0,0);
        transform.setRotation(q);

        untwisted_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/untwisted_cloud", 1);
        untwisted_point_cloud2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/untwisted_cloud2", 1);

        using std::placeholders::_1;
        scan_subscription_ =  this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1, std::bind(&LidarOdometer::scan_callback, this, _1));

    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstPtr& scan) {
        vector<ScanLine<float>> lines;
        ros_scan_to_scan_lines(*scan, lines);
        scan_xy = get_scan_xy(lines);
        auto untwisted = scan_xy;
        if(last_scan_xy.size() > 0) {

            // match the scans with twist
            ScanMatch<float> match;
            {
                Pose<float> diff; // start with zero diff, todo: use velicity / odom to estimate
                const int scans_per_match = 1;

                match = match_scans(last_scan_xy, untwisted, diff);
                diff = match.delta;
                for(uint32_t i = 0; i < 0; ++i) {
                    match = match_scans(last_scan_xy, untwisted, diff);
                    untwisted = untwist_scan<float>(
                        scan_xy, 
                        diff.get_x()/scans_per_match, 
                        diff.get_y()/scans_per_match, 
                        diff.get_theta()/scans_per_match);
                }
            }

            // publish untwisted as a point cloud
            {
                static sensor_msgs::msg::PointCloud point_cloud;

                rclcpp::Time time(scan->header.stamp);
                point_cloud.channels.resize(0);
                point_cloud.header = scan->header;
                point_cloud.points.resize(untwisted.size());
                for(uint32_t i = 0; i < untwisted.size(); ++i) {
                point_cloud.points[i].x = untwisted[i].x;  
                point_cloud.points[i].y = untwisted[i].y;
                point_cloud.points[i].z = 0;
                }
                // untwisted_point_cloud_publisher_.publish(point_cloud);

                // write again as PointCloud2 as that is what many tools expect
                static sensor_msgs::msg::PointCloud2 point_cloud2;
                sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
                untwisted_point_cloud2_publisher_->publish(point_cloud2);
            }
            
            RCLCPP_DEBUG(this->get_logger(), "dx: %f dy: %f dtheta: %f: score: %f", match.delta.get_x(), match.delta.get_y(), match.delta.get_theta(), match.score);

            tf2::Quaternion q;
            // todo: negative here is suspicious, probably need to introduce lidar frame
            q.setRPY(0, 0, match.delta.get_theta());
            tf2::Transform t;
            t.setRotation(q);
            t.setOrigin({match.delta.get_x(), match.delta.get_y(), 0.0});

            transform *= t;

            
  //          auto stamped = tf2::Stamped<tf2::Transform>(transform, tf2::getTimestamp(scan->header.stamp), "map");

            {
                auto r = transform.getRotation();
                tf_msg.transform.rotation.w = r.getW();
                tf_msg.transform.rotation.x = r.getX();
                tf_msg.transform.rotation.y = r.getY();
                tf_msg.transform.rotation.z = r.getZ();

                auto t = transform.getOrigin();
                tf_msg.transform.translation.x = t.getX();
                tf_msg.transform.translation.y = t.getY();
                tf_msg.transform.translation.z = t.getZ();

                tf_msg.header.stamp = scan->header.stamp;
            }



//            tf_msg = tf2_ros::toMsg<tf2::Stamped<tf2::Transform>, geometry_msgs::msg::TransformStamped>(stamped);
            tf_msg.header.frame_id = "lidar_odom";
            tf_msg.child_frame_id = "lidar_odom_laser";

            static tf2_ros::TransformBroadcaster br(this);
            br.sendTransform(tf_msg);
        }
        last_scan_xy = untwisted;
    }

};

#include <iostream>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LidarOdometer>());
    rclcpp::shutdown();
    return 0;
}