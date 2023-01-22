#ifndef IMAGES_CONNECTOR_H_
#define IMAGES_CONNECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace dom_estimator
{
class ImageSubscriber
{
public:
    ImageSubscriber();
    ImageSubscriber(ros::NodeHandle _nh,std::string robot_name);

    cv::Mat get_img();

private:
    void img_callback(const sensor_msgs::ImageConstPtr& msg);

    // node handle
    ros::NodeHandle nh_;

    // subscriber
    ros::Subscriber img_sub_;

    // buffer
    cv::Mat img_;
};

class ImagesConnector : public std::vector<ImageSubscriber*>
{
public:
    ImagesConnector();
    void process();

private:
    void init();
    void publish_img();

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher img_pub_;

    // param
    int HZ_;
};
} // namespace dom_estimator

#endif	// IMAGES_CONNECTOR_H_