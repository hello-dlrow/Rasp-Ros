#ifndef LINE_DETECT_NODE_HPP
#define LINE_DETECT_NODE_HPP

#include <array>
#include <bits/stdint-uintn.h>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <sys/types.h>

class LineDetect {
public:
  LineDetect(const std::uint16_t &height, const std::uint16_t &width,
             const std::uint8_t op);
  ~LineDetect();

  cv::Mat image;

  void processImage();

private:
  uint16_t height;
  uint16_t width;
  uint8_t op;

  int xCord;
  int yCord;

  std::array<cv::Point, 480> leftPoints;
  std::array<cv::Point, 480> rightPoints;
  cv::Scalar lower_color;
  cv::Scalar higher_color;
  cv::Mat binary_mask;
  /*
  region of interst function
  */

  void cvtImg(cv::Mat &image);
  void binaryImg(cv::Mat &image);
  void findMaxline(cv::Mat &image);
  std::vector<int> findBottomWhite(cv::Mat &image);
};

#endif
