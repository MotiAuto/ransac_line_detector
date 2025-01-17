#ifndef RANSAC_LINE_DETECTOR_HPP_
#define RANSAC_LINE_DETECTOR_HPP_

#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "ransac.hpp"

using std::placeholders::_1;

namespace ransac_line_detector
{
    class RansacLineDetector : public rclcpp::Node
    {
        public:
        explicit RansacLineDetector(const rclcpp::NodeOptions& option);

        void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

        int detect_num, max_iter_;
        double threshold_;
        std::shared_ptr<RANSAC> ransac;
        visualization_msgs::msg::Marker marker;
        std::string frame_id;
    };

    void PointCloudControl(const LineModel &line, const double &threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::PointXYZ *start, pcl::PointXYZ *end);

    geometry_msgs::msg::Point PCLtoPoint(const pcl::PointXYZ &p);
}

#endif