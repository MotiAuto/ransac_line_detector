#ifndef RANSAC_HPP_
#define RANSAC_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

        LineModel segment(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        private:
        int max_iteration_;
        double threshold_;
    };

    LineModel computeLine(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

    double point2lineDistance(const pcl::PointXYZ& p, const LineModel& line);
}

#endif