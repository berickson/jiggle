#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Transform.h>

#include "lidar_mapper.h"


class LidarOdometer {
public:
    ros::NodeHandle n;
    int seq = 0;
    vector<Point2d<float>> scan_xy;
    vector<Point2d<float>> last_scan_xy;
    geometry_msgs::TransformStamped tf_msg;
    tf2::Transform transform;

    LidarOdometer() {
        transform.setOrigin({0.0,0.0,0.0});
        tf2::Quaternion q;
        q.setRPY(0,0,0);
        transform.setRotation(q);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<ScanLine<float>> lines;
        ros_scan_to_scan_lines(*scan, lines);
        scan_xy = get_scan_xy(lines);
        if(last_scan_xy.size() > 0) {
            ++seq;
            Pose<float> guess;
            auto match = match_scans(last_scan_xy, scan_xy, guess);
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
        last_scan_xy = scan_xy;
    }

    void run() {
        ROS_INFO("starting run");
        ros::Subscriber sub = n.subscribe("scan", 1, &LidarOdometer::scan_callback, this);

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