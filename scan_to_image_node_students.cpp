#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iostream>
#include <cmath>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node")
    {
        // Subscription to map and scan topics
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        // Pose of the robot (initial position known)
        initial_pose_.position.x = 0.0;
        initial_pose_.position.y = 0.0;
        initial_pose_.orientation.w = 1.0; // No initial rotation

        // window to display the map and scan
        cv::namedWindow("LaserScan", cv::WINDOW_AUTOSIZE);
    }

private:
    // Callback for receiving LaserScan data
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg) {
        cv::Mat scan_img = laserScanToImage(scanMsg); // Convert LaserScan to image (Image C)
        
        if (!m_MapColImage.empty()) {
            // Compare Image B (edges from the map) with Image C (edges from LaserScan)
            double angle = compareImagesForRotation(map_edges_, scan_img);
            std::cout << "Estimated rotation: " << angle << " degrees\n";

            // Update pose based on odometry and estimated angle
            updateRobotPose(angle);

            // Display scan image
            cv::imshow("LaserScan", scan_img);
            cv::waitKey(1);
        }
    }

    // Callback for receiving the map data
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        occupancyGridToImage(mapMsg);

        // Moved origin_x, origin_y, and map_scale_ inside the function
        origin_x = mapMsg->info.origin.position.x;
        origin_y = mapMsg->info.origin.position.y;
        map_scale_ = mapMsg->info.resolution;

        // Extract a section of the map around the robot (Image A)
        cv::Rect roi = extractMapSection(mapMsg, initial_pose_);
        cv::Mat map_section = m_MapColImage(roi);

        // Create an edge image from the map section (Image B)
        cv::Mat edges;
        cv::Canny(map_section, edges, 50, 150);
        map_edges_ = edges.clone();

        // Display the map section and edges
        cv::imshow("Map Section", map_section);
        cv::imshow("Map Edges", map_edges_);
        cv::waitKey(1);
    }

    // Convert OccupancyGrid to an OpenCV image
    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        m_MapColImage = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        for (unsigned int row = 0; row < grid->info.height; row++) {
            for (unsigned int col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    unsigned int val = 255 - (255 * grid_data) / 100;
                    m_MapColImage.at<uchar>(grid->info.height - row - 1, col) = (val == 0) ? 255 : 0;
                } else {
                    m_MapColImage.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }
    }

    // Extract a section of the map around the robot's position
    cv::Rect extractMapSection(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg, const geometry_msgs::msg::Pose& pose)
    {
        int x = static_cast<int>((pose.position.x - origin_x) / map_scale_);
        int y = static_cast<int>((pose.position.y - origin_y) / map_scale_);

        int section_size = 100; // Define the size of the section (e.g., 100x100 pixels)

        int x_start = std::max(0, x - section_size / 2);
        int y_start = std::max(0, y - section_size / 2);
        int x_end = std::min(m_MapColImage.cols, x + section_size / 2);
        int y_end = std::min(m_MapColImage.rows, y + section_size / 2);
    
        // Adjust ROI because the map is out of bounds
        int roi_width = x_end - x_start;
        int roi_height = y_end - y_start;

        return cv::Rect(x - section_size / 2, y - section_size / 2, section_size, section_size);
    }

    double origin_x, origin_y, map_scale_;

    // Convert LaserScan to an image (Image C)
    cv::Mat laserScanToImage(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int img_size = 500;
        cv::Mat scan_image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        float max_range = scan->range_max;
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    scan_image.at<uchar>(y, x) = 255;
                }
            }
        }
        return scan_image;
    }

    // Compare two images (Image B and Image C) to estimate the rotation
    double compareImagesForRotation(const cv::Mat& img1, const cv::Mat& img2)
    {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(img1, img2, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for transformation.");
            return 0.0;
        }

        // Estimate rotation from matched points
        cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
        if (!transform_matrix.empty()) {
            double angle = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
            return angle * 180.0 / CV_PI;
        }
        return 0.0;
    }

    // Detect and match features between two images
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    // Update the robot's pose based on odometry and estimated angle
    void updateRobotPose(double angle)
    {
        initial_pose_.orientation.w = cos(angle / 2.0);
        initial_pose_.orientation.z = sin(angle / 2.0);
        RCLCPP_INFO(this->get_logger(), "Updated robot orientation: %f degrees", angle);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    geometry_msgs::msg::Pose initial_pose_;
    cv::Mat m_MapColImage, map_edges_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}