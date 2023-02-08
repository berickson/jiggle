#include <sensor_msgs/msg/point_cloud2.hpp>
#include "dewarp.h"

void set_point_cloud_points(sensor_msgs::msg::PointCloud2 &p, const std::vector<Point2d<float>> & points) {
    p.height = 1;
    p.width = points.size();
    p.fields.resize(3);
    p.fields[0].name = "x";
    p.fields[1].name = "y";
    p.fields[2].name = "z";
    int offset = 0;
    for (size_t d = 0; d < p.fields.size(); ++d, offset += sizeof(float)) {
        p.fields[d].offset = offset;
        p.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
        p.fields[d].count = 1;
    }
    p.point_step = offset;
    p.row_step = p.point_step * p.width;
    p.data.resize(p.width * p.point_step);
    p.is_bigendian = false;
    p.is_dense = false;
    // Copy the data points
    float z = 0.0;
    for (size_t cp = 0; cp < p.width; ++cp) {
        memcpy(
            &p.data[cp * p.point_step + p.fields[0].offset],
            &points[cp].x, sizeof(float));
        memcpy(
            &p.data[cp * p.point_step + p.fields[1].offset],
            &points[cp].y, sizeof(float));
        memcpy(
            &p.data[cp * p.point_step + p.fields[2].offset],
            &z, sizeof(float));
    }
}