#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/point_cloud_conversion.h"

#include <tf2/LinearMath/Transform.h>

#include "lidar_mapper.h"


class LidarOdometer {
public:
    ros::NodeHandle node_;
    int seq = 0;
    vector<Point2d<float>> scan_xy;
    vector<Point2d<float>> last_scan_xy;
    geometry_msgs::TransformStamped tf_msg;
    tf2::Transform transform;

    ros::Publisher untwisted_point_cloud_publisher_;
    ros::Publisher untwisted_point_cloud2_publisher_;

    LidarOdometer() {
        transform.setOrigin({0.0,0.0,0.0});
        tf2::Quaternion q;
        q.setRPY(0,0,0);
        transform.setRotation(q);

        untwisted_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("/untwisted_cloud", 1, false);
        untwisted_point_cloud2_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/untwisted_cloud2", 1, false);
    }


    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<ScanLine<float>> lines;
        ros_scan_to_scan_lines(*scan, lines);
        scan_xy = get_scan_xy(lines);
        auto untwisted = scan_xy;
        if(last_scan_xy.size() > 0) {
            ++seq;

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
                static sensor_msgs::PointCloud point_cloud;
                ros::Time time(scan->header.stamp);
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
                static sensor_msgs::PointCloud2 point_cloud2;
                sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
                untwisted_point_cloud2_publisher_.publish(point_cloud2);
            }
            
            ROS_DEBUG("dx: %f dy: %f dtheta: %f: score: %f", match.delta.get_x(), match.delta.get_y(), match.delta.get_theta(), match.score);

            tf2::Quaternion q;
            // todo: negative here is suspicious, probably need to introduce lidar frame
            q.setRPY(0, 0, match.delta.get_theta());
            tf2::Transform t;
            t.setRotation(q);
            t.setOrigin({match.delta.get_x(), match.delta.get_y(), 0.0});

            transform *= t;

            auto stamped = tf2::Stamped<tf2::Transform>(transform, scan->header.stamp, "map");
            tf_msg = tf2::toMsg(stamped);
            tf_msg.header.frame_id = "lidar_odom";
            tf_msg.header.seq = seq;
            tf_msg.child_frame_id = "lidar_odom_laser";

            static tf2_ros::TransformBroadcaster br;
            br.sendTransform(tf_msg);
        }
        last_scan_xy = untwisted;
    }

    void run() {
        ROS_INFO("starting run");
        ros::Subscriber sub = node_.subscribe("scan", 1, &LidarOdometer::scan_callback, this);

        ros::spin();

    }
};

#include <iostream>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "lidar_odom");
    ROS_INFO("starting LidarOdometer");
    LidarOdometer lidar_odometer;
    lidar_odometer.run();
    std::cout << std::endl <<  "exiting lidar_odom" << std::endl;
    return 0;
}