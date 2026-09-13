#include "perception.hpp"

// ROS Headers, ros namespace
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    // "spin" blocks until our node dies
    rclcpp::spin(std::make_shared<mrover_autonomy_starter::Perception>());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

namespace mrover_autonomy_starter {

    // Constructor for the perception() node
    Perception::Perception() : Node("perception") {
        // Subscriber to the input images from the ZED camera topic with a queue size of 1.
        // Every time a node publishes to /zed/left/image, our inline lambda callback forwards frames to the imageCallback() method for processing.
        mImageSubscriber = create_subscription<sensor_msgs::msg::Image>("/zed/left/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& frame) {
            imageCallback(frame);
        });

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        // mTagPublisher = create_publisher<msg::StarterProjectTag>("tag", 1);

        // Store the 50 valid 4x4 IDs (0-49) in a class cv::Ptr variable, extending the lifetime of the dictionary beyond this constructor 
        // and lets detectMarkers() reuse it every frame without copying it.
        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }

    auto Perception::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& imageMessage) -> void {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a small header that points to the actual image data
        cv::Mat imageBGRA{static_cast<int>(imageMessage->height), static_cast<int>(imageMessage->width),
                CV_8UC4, const_cast<uint8_t*>(imageMessage->data.data())};
        cv::Mat image;

        // Convert from BGRA to BGR by removing the alpha (transparency) channel since it isn't used
        cv::cvtColor(imageBGRA, image, cv::COLOR_BGRA2BGR);

        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        (void)this;
    }

    auto Perception::findTagsInImage(cv::Mat const& image) -> void { // NOLINT(*-convert-member-functions-to-static)
        // Take a look at OpenCV's documentation: https://docs.opencv.org/4.5.0/d5/dae/tutorial_aruco_detection.html
        // You have mTagDictionary, mTagCorners, and mTagIds member variables already defined!
        // You might want to call getCenterFromTagCorners() and getClosenessMetricFromTagCorners() within this function

        mTags.clear(); // Clear old tags in output vector, since mTags persists across each imageCallback() call as a class variable.

        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        (void)image;
    }

    auto Perception::selectTag(std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        // If there isn't a valid tag, you should return a "dummy" tag with ID -1.
        (void)tags;

        msg::StarterProjectTag noTag{};
        noTag.tag_id = -1;
        return noTag;
    }

    auto Perception::publishTag(msg::StarterProjectTag const& tag) -> void {
        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        (void)tag;
    }

    auto Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me! Calculate and return a closeness metric for this tag.
        // The exact calculation is your choice, but it must:
        //   - return a finite value between 0.0F and 1.0F;
        //   - produce smaller values for closer/larger-looking tags;
        //   - produce larger values for farther/smaller-looking tags.
        //
        // HINT: A nearby tag generally occupies more pixels in the image. Consider using the both image and tag dimensions.
        // Read the function header in perception.hpp for more hints.
        (void)image;
        (void)tagCorners;

        return 1.0F;
    }  

    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        (void)tagCorners;

        return {0.0F, 0.0F};
    }

} // namespace mrover_autonomy_starter