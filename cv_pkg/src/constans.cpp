#include <cstdint>
#include <opencv2/opencv.hpp>

namespace constants {
constexpr std::uint16_t height = 680;
constexpr std::uint16_t width = 480;
cv::Scalar lower_color_black(0, 0, 0);
cv::Scalar higher_color_black(180, 255, 20);
} // namespace constants

std::uint16_t getHeight { return constants::height; }
std::uint16_t getWidth { return constants::width; }
cv::Scalar getLowerBck { return constants::lower_color_black; }
cv::Scalar getHigherBck { return constants::higher_color_black; }
