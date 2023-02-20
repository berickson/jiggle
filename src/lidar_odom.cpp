#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <laser_geometry/laser_geometry.hpp> 

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer_interface.h>

#include <vector>

#include "lidar_mapper.h"
#include "point_cloud_helper.h"

using namespace std;

class LidarOdometer : public rclcpp::Node {
public:
    vector<Point2d<float>> scan_xy;
    vector<Point2d<float>> last_scan_xy;
    tf2::Transform transform;
    Pose<float> pose;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr untwisted_point_cloud2_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

    LidarOdometer() 
    : Node("lidar_odometer")
    {

        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "True if the input scan will be warped due to movement and should be dewarped";
            this->declare_parameter("dewarp", true, param_desc);
        }

        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "The number of times to dewarp and refine matching, only used if dewarp is True";
            param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
            
            rcl_interfaces::msg::IntegerRange range;
            range.set__from_value(1).set__to_value(10).set__step(1);
            param_desc.integer_range= {range};

            this->declare_parameter("dewarp_iterations", 2, param_desc);
        }

        transform.setOrigin({0.0,0.0,0.0});
        tf2::Quaternion q;
        q.setRPY(0,0,0);
        transform.setRotation(q);

        untwisted_point_cloud2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/untwisted_cloud", 1);

        using std::placeholders::_1;
        scan_subscription_ =  this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1, std::bind(&LidarOdometer::scan_callback, this, _1));

    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) {
        lidar_odom_timer.start();
        vector<ScanLine<float>> lines;
        ros_scan_to_scan_lines(*scan, lines);
        scan_xy = get_scan_xy(lines);
        auto untwisted = scan_xy;
        if(last_scan_xy.size() > 0) {

            // match the scans with twist
            ScanMatch<float> match;
            {
                bool dewarp = this->get_parameter("dewarp").get_parameter_value().get<bool>();
                int32_t dewarp_iterations = dewarp ? this->get_parameter("dewarp_iterations").get_parameter_value().get<int32_t>() : 1;
                Pose<float> diff; // start with zero diff, todo: use velicity / odom to estimate
                const int scans_per_match = 1;

                for(uint32_t i = 1; i <= dewarp_iterations; ++i) {
                    match = match_scans(last_scan_xy, untwisted, diff);
                    diff = match.delta;
                    if(dewarp) {
                        untwisted = untwist_scan<float>(
                            scan_xy, 
                            diff.get_x()/scans_per_match, 
                            diff.get_y()/scans_per_match, 
                            diff.get_theta()/scans_per_match);
                    } else {
                        break;
                    }
                }
            }

            // publish untwisted as a point cloud
            static sensor_msgs::msg::PointCloud2 p;
            p.header = scan->header;
            set_point_cloud_points(p, untwisted);
            if(untwisted_point_cloud2_publisher_->get_subscription_count() > 0)
            {
            //     p.height = 1;
            //     p.width = untwisted.size();
            //     p.fields.resize(3);
            //     p.fields[0].name = "x";
            //     p.fields[1].name = "y";
            //     p.fields[2].name = "z";
            //     int offset = 0;
            //     for (size_t d = 0; d < p.fields.size(); ++d, offset += sizeof(float)) {
            //         p.fields[d].offset = offset;
            //         p.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            //         p.fields[d].count = 1;
            //     }
            //     p.point_step = offset;
            //     p.row_step = p.point_step * p.width;
            //     p.data.resize(p.width * p.point_step);
            //     p.is_bigendian = false;
            //     p.is_dense = false;
            //     // Copy the data points
            //     float z = 0.0;
            //     for (size_t cp = 0; cp < p.width; ++cp) {
            //         memcpy(
            //             &p.data[cp * p.point_step + p.fields[0].offset],
            //             &untwisted[cp].x, sizeof(float));
            //         memcpy(
            //             &p.data[cp * p.point_step + p.fields[1].offset],
            //             &untwisted[cp].y, sizeof(float));
            //         memcpy(
            //             &p.data[cp * p.point_step + p.fields[2].offset],
            //             &z, sizeof(float));
            //     }


                untwisted_point_cloud2_publisher_->publish(p);
            }
            
            RCLCPP_DEBUG(this->get_logger(), "dx: %f dy: %f dtheta: %f: score: %f", match.delta.get_x(), match.delta.get_y(), match.delta.get_theta(), match.score);

            pose.move({match.delta.get_x(), match.delta.get_y()}, match.delta.get_theta());

            tf2::Quaternion q;
            q.setRPY(0, 0, match.delta.get_theta());
            tf2::Transform t;
            t.setRotation(q);
            t.setOrigin({match.delta.get_x(), match.delta.get_y(), 0.0});

            transform *= t;

            RCLCPP_INFO(this->get_logger(), "pose: {%f},{%f},{%f} transform: {%f},{%f},{%f}", 
                pose.get_x(), 
                pose.get_y(), 
                pose.get_theta(), 
                transform.getOrigin().getX(),
                transform.getOrigin().getY(),
                transform.getRotation().getAngle());

            geometry_msgs::msg::TransformStamped tf_msg;
            {

                tf_msg.header.frame_id = "lidar_odom";
                tf_msg.child_frame_id = "lidar_odom_laser";
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




            static tf2_ros::TransformBroadcaster br(this);
            br.sendTransform(tf_msg);

            // send empty transform from map to lidar_odom
            {
                geometry_msgs::msg::TransformStamped map_to_lidar_msg;
                map_to_lidar_msg.header.stamp = tf_msg.header.stamp;
                map_to_lidar_msg.header.frame_id = "map";
                map_to_lidar_msg.child_frame_id = "lidar_odom";
                map_to_lidar_msg.transform.rotation.w=1;
                map_to_lidar_msg.transform.rotation.x=0;
                map_to_lidar_msg.transform.rotation.y=0;
                map_to_lidar_msg.transform.rotation.z=0;
                map_to_lidar_msg.transform.translation.x=0;
                map_to_lidar_msg.transform.translation.y=0;
                map_to_lidar_msg.transform.translation.z=0;
                br.sendTransform(map_to_lidar_msg);
            }
        }
        last_scan_xy = untwisted;
        lidar_odom_timer.stop();
    }

};

#include <iostream>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LidarOdometer>());
    rclcpp::shutdown();
    return 0;
}