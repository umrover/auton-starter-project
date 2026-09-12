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

    Perception::Perception() : Node("perception") {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = create_subscription<sensor_msgs::msg::Image>("zed/left/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            imageCallback(msg);
        });

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = create_publisher<msg::StarterProjectTag>("tag", 1);

        // In order for future calls to cv::aruco::detectMarkers to work, we must first get the ArUco dictionary for 4x4 tags with ids from 0-49.
        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }

    auto Perception::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& imageMessage) -> void {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat imageBGRA{static_cast<int>(imageMessage->height), static_cast<int>(imageMessage->width),
                      CV_8UC4, const_cast<uint8_t*>(imageMessage->data.data())};
        cv::Mat image;

        cv::cvtColor(imageBGRA, image, cv::COLOR_BGRA2BGR);

        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        // find tags in image
        findTagsInImage(image);
        
        // call publishTag
        publishTag(selectTag(mTags));
    }

    auto Perception::findTagsInImage(cv::Mat const& image) -> void { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, and mTagIds member variables already defined! (look in perception.hpp)
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        mTags.clear(); // Clear old tags in output vector

        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        
        // populate mTagCorners and mTagIds
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds);
        for (uint i = 0; i < mTagIds.size(); i++){
            int tagId = mTagIds[i];
            auto tagCenter = getCenterFromTagCorners(mTagCorners[i]);
            auto closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);

            msg::StarterProjectTag tagMsg;
            tagMsg.tag_id = tagId;
            tagMsg.x_tag_center_pixel = tagCenter.first;
            tagMsg.y_tag_center_pixel = tagCenter.second;
            tagMsg.closeness_metric = closenessMetric;
            mTags.push_back(tagMsg);
        }
    }

    auto Perception::selectTag(std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        // If there isn't a valid tag, you should return a "dummy" tag with ID -1.
        if(tags.empty()){
            msg::StarterProjectTag noTag;
            noTag.tag_id = -1;
            return noTag;
        }  

        auto minSeen = tags[0].closeness_metric;
        auto closest = tags[0];

        for(uint i = 1; i < tags.size(); i++){
            if(tags[i].closeness_metric < minSeen){
                closest = tags[i];
                minSeen = tags[i].closeness_metric;
            }
        }

        return closest;
    }

    auto Perception::publishTag(msg::StarterProjectTag const& tag) -> void {
        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        mTagPublisher->publish(tag);

    }

    auto Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        // find the area of the tag, and divide by the total area of the image
        // tagCorners 0: top left, 1: top right, 2: bottom right, 3: bottom left
        auto imageHeight = image.rows;
        auto imageWidth = image.cols;

        auto imageArea = imageHeight * imageWidth;
        
        auto topWidth = std::abs(tagCorners[0].x-tagCorners[1].x);
        auto bottomWidth = std::abs(tagCorners[2].x-tagCorners[3].x);
        auto avgWidth = (topWidth + bottomWidth)/2;
        
        auto leftHeight = std::abs(tagCorners[0].y-tagCorners[3].y);
        auto rightHeight = std::abs(tagCorners[1].y-tagCorners[2].y);
        auto avgHeight = (leftHeight+rightHeight)/2;

        auto avgArea = avgWidth*avgHeight;

        return (1-std::sqrt(avgArea/static_cast<float>(imageArea)));
    }

    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me! Read the wiki and the function header in perception.hpp for more hints.
        std::pair<float,float> center;
        center.first = (tagCorners[0].x + tagCorners[1].x + tagCorners[2].x + tagCorners[3].x)/static_cast<float>(4.0);
        center.second = (tagCorners[0].y + tagCorners[1].y + tagCorners[2].y + tagCorners[3].y)/static_cast<float>(4.0);
        return center;
    }

} // namespace mrover_autonomy_starter