/*
Suscribe camera image topic
Process image frome camera
Return deviation from mid line
Return front vision length
*/

#include "line_detect_node.hpp"

#include <bits/stdint-uintn.h>
#include <cv_bridge/cv_bridge.h>
#include <endian.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

constexpr std::uint16_t height = 680;
constexpr std::uint16_t width = 480;
cv::Scalar lower_color_black(0, 0, 0);
cv::Scalar higher_color_black(180, 255, 20);

void Cam_RGB_Callback(const sensor_msgs::Image msg) {

  cv_bridge::CvImagePtr cv_ptr;
  LineDetect detector(height, width, 1);

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat imgOriginal = cv_ptr->image;
  // imgOriginal为原始帧，所有的处理基于此帧
  cv::imshow("BGR", imgOriginal);
  cv::waitKey(1);

  //将原始帧拷贝进巡线类中的私有成员
  detector.image = imgOriginal.clone();
  detector.processImage();
  cv::imshow("Binary Image", detector.image);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_detect_node");
  ros::NodeHandle nh;

  ros::Subscriber rgb_sub = nh.subscribe("camera/image", 1, Cam_RGB_Callback);

  //显示原始图像
  cv::namedWindow("BGR");
  //显示二值化后的图像
  cv::namedWindow("Binary Image");
  ros::spin();

  return 0;
}

LineDetect::LineDetect(const std::uint16_t &height, const std::uint16_t &width,
                       const std::uint8_t op)
    : height(height), width(width), op(op) {
  lower_color = lower_color_black;
  higher_color = higher_color_black;
}
// Leave empty
LineDetect::~LineDetect(){};

//转换到符合人眼色彩空间的hsv
void LineDetect::cvtImg(cv::Mat &image) {
  cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
}

//根据赛道颜色二值化图像
void LineDetect::binaryImg(cv::Mat &image) {
  cv::inRange(image, lower_color, higher_color, binary_mask);
  image = binary_mask;
}

//图像处理函数封装，提供一个公共接口
void LineDetect::processImage() {
  cvtImg(image);
  binaryImg(image);
}

//处理二值化图像，利用最长黑线法寻找赛道
//这个函数负责寻找最长的黑线
uint16_t findMaxline(cv::Mat &image) { return 1; }
