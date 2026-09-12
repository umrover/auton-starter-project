#pragma once

// C++ Standard Library Headers, std namespace
#include <memory>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// OpenCV Headers, cv namespace
#include <opencv2/core/mat.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

// ROS Headers, ros namespace
# include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mrover_autonomy_starter/msg/starter_project_tag.hpp>

namespace mrover_autonomy_starter {

    /**
     *  Starter project perception node
     *
     *  Input:  RGB pixel image data
     *  Output: StarterProjectTag message containing the tag ID, center coordinates, and closeness metric of the ArUco tag closest to the camera
     */
    class Perception : public rclcpp::Node {
    private:
        // Store pointer to subscriber that receives ZED camera frames
        rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr mImageSubscriber;

        // Detector holding the ArUco dictionary used for marker detection, constructed once and reused every frame
        cv::aruco::ArucoDetector mTagDetector;

        // Vector to store the detected corners in the current frame
        std::vector<std::vector<cv::Point2f>> mTagCorners;

        // Vector to stores the detected IDs in the current frame
        std::vector<int> mTagIds;

        // Vector to store detected tags and their information
        std::vector<msg::StarterProjectTag> mTags;

        // Store pointer to publisher for selected tag
        rclcpp::Publisher<msg::StarterProjectTag>::SharedPtr mTagPublisher;

    public:
        Perception();

        /**
         * Called when we receive a new image message (a new frame) from the camera.
         *
         * @param imageMessage
         */
        void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& imageMessage);

        /**
         *  Given an image, detect ArUco tags, and fill mTags full of output messages.
         *
         * @param image The image from the camera
         */
        void findTagsInImage(cv::Mat const& image);

        /**
         * Publish the closest tag
         *
         * @param tag The selected tag message to publish
         */
        void publishTag(msg::StarterProjectTag const& tag);

        /**
         *  Given an ArUco tag in pixel space, find a metric for how close we are.
         *
         * @param image         The raw RGB image
         * @param tagCorners    4 (x,y) pairs representing the coordinates of each corners
         * @return              A closeness metric of the tag (should be between 0 and 1, where 0 is closest, 1 is farthest)
         */
        [[nodiscard]] auto getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float;

        /**
         *  Given an ArUco tag in pixel space, find the approximate center in pixel space
         *
         * @param tagCorners    4 (x,y) pairs representing the pixel coordinates of each corner
         * @return              The (x,y) approximate center of the tag in pixel space
         */
        [[nodiscard]] auto getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float>;

        /**
         *  Select the tag closest to the camera. If there isn't any tags, return a "dummy" tag with ID of -1.
         * 
         * @param tags          Tags detected in the current frame.
         * @return              The tag with the lowest closeness metric, or a tag with ID -1 if no tags were detected.
         */
        [[nodiscard]] auto selectTag(std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag;
    };

} // namespace mrover_autonomy_starter