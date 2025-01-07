#include "ransac_line_detector/ransac_line_detector.hpp"

namespace ransac_line_detector
{
    RansacLineDetector::RansacLineDetector(const rclcpp::NodeOptions& option): Node("RansacLineDetector", option)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud",
            qos_settings,
            std::bind(&RansacLineDetector::topic_callback, this, _1)
        );

        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lines", rclcpp::SystemDefaultsQoS());

        this->declare_parameter("detect_num", 1);
        this->get_parameter("detect_num", detect_num);
        this->declare_parameter("max_iter_num", 200);
        this->get_parameter("max_iter_num", max_iter_);
        this->declare_parameter("threshold", 0.1);
        this->get_parameter("threshold", threshold_);

        ransac = std::make_shared<RANSAC>(max_iter_, threshold_);

        marker.header.frame_id = "base_link";
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        RCLCPP_INFO(this->get_logger(), "Start RansacLineDetector");
    }

    void RansacLineDetector::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        auto marker_array = visualization_msgs::msg::MarkerArray();

        for(int i = 0; i < detect_num; i++)
        {
            pcl::PointXYZ line_start, line_end;
            auto line = ransac->segment(cloud);

            PointCloudControl(line, 0.1, cloud, &line_start, &line_end);

            marker.points.push_back(PCLtoPoint(line_start));
            marker.points.push_back(PCLtoPoint(line_end));

            marker_array.markers.push_back(marker);
        }

        pub_->publish(marker_array);
    }

    void PointCloudControl(const LineModel &line, const double &threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::PointXYZ *start, pcl::PointXYZ *end)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ max, min;

        for(const auto& p : *pointcloud)
        {
            if(point2lineDistance(p, line) > threshold)
            {
                cloud->push_back(p);
            }
            else
            {
                if(max.x < p.x && max.y < p.y)
                {
                    max = p;
                }
                else if(min.x > p.x && min.y > p.y)
                {
                    min = p;
                }
            }
        }

        pointcloud = cloud;
        *start = min;
        *end = max;
    }

    geometry_msgs::msg::Point PCLtoPoint(const pcl::PointXYZ &p)
    {
        geometry_msgs::msg::Point p1;
        p1.x = p.x;
        p1.y = p.y;
        p1.z = p.z;

        return p1;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ransac_line_detector::RansacLineDetector)