/*
Suscribe camera image topic
Process image frome camera
Return deviation from mid line
Return front vision length
*/
#include "line_detect_node.hpp"
#include "constans.h"

#include <algorithm>
#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <cv_bridge/cv_bridge.h>
#include <endian.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ostream>
#include <ros/ros.h>
#include <sys/types.h>

void Cam_RGB_Callback(const sensor_msgs::Image msg) {

  cv_bridge::CvImagePtr cv_ptr;
  LineDetect detector(constants::height, constants::width, 1);

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
    : height(constants::height), width(constants::width), op(1) {
  lower_color = constants::lower_color_black;
  higher_color = constants::higher_color_black;
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

std::vector<int> LineDetect::findBottomWhite(cv::Mat &image) {
  int rows = image.rows;
  int cols = image.cols;

  int bottomRow = rows - 1;
  int centerCol = cols / 2;

  std::vector<int> result = {0, cols};

  for (int l = centerCol; l >= 0; --l) {
    if (image.at<uchar>(bottomRow, l) == 255) {
      result[0] = l;
      break;
    }
  }
  for (int r = centerCol; r < cols; ++r) {
    if (image.at<uchar>(bottomRow, r) == 255) {
      result[1] = r;
      break;
    }
  }
  return result;
}

//处理二值化图像，利用最长黑线法寻找赛道
//这个函数负责寻找最长的黑线
void LineDetect::findMaxline(cv::Mat &image) {
  int rows = image.rows;
  int cols = image.cols;

  int globalMaxlen = 0;
  int x, y;

  std::vector<int> whitebottom = findBottomWhite(image);
  if (whitebottom[0] == whitebottom[1] && whitebottom[0] == cols / 2) {
    whitebottom[0]--;
    whitebottom[1]++;
  }
  for (int c = whitebottom[0]; c < whitebottom[1]; ++c) {

    int currentLen = 0;
    int maxLenInThisCol = 0;

    for (int r = rows; r > 0; --r) {
      uchar pixelVal = image.at<uchar>(r, c);
      if (pixelVal == 0) {
        currentLen++;
      } else {
        maxLenInThisCol = std::max(maxLenInThisCol, currentLen);
        if (maxLenInThisCol == currentLen) {
          y = r;
        }
        currentLen = 0;
      }
    }

    maxLenInThisCol = std::max(maxLenInThisCol, currentLen);
    if (maxLenInThisCol == currentLen) {
      y = rows;
    }
    globalMaxlen = std::max(globalMaxlen, maxLenInThisCol);
    if (globalMaxlen == maxLenInThisCol) {
      x = c;
    }
  }
  this->xCord = x;
  this->yCord = y;

  std::cout << "X:" << this->xCord << " Y:" << this->yCord
            << " Length:" << globalMaxlen << " Left:" << whitebottom[0]
            << " Right" << whitebottom[1] << std::endl;
}

//图像处理函数封装，提供一个公共接口
void LineDetect::processImage() {
  cvtImg(image);
  binaryImg(image);
  findMaxline(image);
}
