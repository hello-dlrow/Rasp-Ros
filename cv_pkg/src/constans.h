#ifndef CONSTANS_H
#define CONSTANS_H
#include <cstdint>
#include <opencv2/opencv.hpp>

namespace constants {
constexpr std::uint16_t height = 680;
constexpr std::uint16_t width = 480;
cv::Scalar lower_color_black(0, 0, 0);
cv::Scalar higher_color_black(180, 255, 20);
} // namespace constants

#endif
