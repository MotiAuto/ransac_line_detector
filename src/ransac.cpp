#include "ransac_line_detector/ransac.hpp"

namespace ransac_line_detector
{
    RANSAC::RANSAC(int max_iter, double threshold)
    {
        max_iteration_ = max_iter;
        threshold_ = threshold;
    }

    LineModel RANSAC::segment(const sensor_msgs::msg::PointCloud& pointcloud)
    {
        LineModel best_model;
        int max_inlier = 0;

        for(int i = 0; i < max_iteration_; i++)
        {
            int id1 = rand() % pointcloud.points.size();
            int id2 = rand() % pointcloud.points.size();
            if(id1 == id2) continue;

            LineModel line = computeLine(pointcloud.points[id1], pointcloud.points[id2]);

            int inlier = 0;
            for(const auto& p : pointcloud.points)
            {
                if(point2lineDistance(p, line) < threshold_)
                {
                    inlier++;
                }
            }

            if(inlier > max_inlier)
            {
                max_inlier = inlier;
                best_model = line;
            }
        }

        return best_model;
    }

    LineModel computeLine(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2)
    {
        LineModel line;
        line.a = p2.y - p1.y;
        line.b = p1.x - p2.x;
        line.c = p2.x * p1.y - p1.x * p2.y;

        return line;
    }

    double point2lineDistance(const geometry_msgs::msg::Point32& p, const LineModel& line)
    {
        const auto v1 = std::abs(line.a * p.x + line.b * p.y + line.c);
        const auto v2 = std::sqrt(line.a*line.a + line.b*line.b);

        return v1 / v2;
    }
}