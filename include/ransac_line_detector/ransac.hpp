#ifndef RANSAC_HPP_
#define RANSAC_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>

namespace ransac_line_detector
{
    struct LineModel
    {
        double a, b, c;
    };
    

    class RANSAC
    {
        public:
        RANSAC(int max_iter, double threshold);

        LineModel segment(const sensor_msgs::msg::PointCloud& pointcloud);

        private:
        int max_iteration_;
        double threshold_;
    };

    LineModel computeLine(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2);

    double point2lineDistance(const geometry_msgs::msg::Point32& p, const LineModel& line);
}

#endif