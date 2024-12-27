#ifndef VIDEO_STREAM_HPP
#define VIDEO_STREAM_HPP

#include <arpa/inet.h> // 添加这个用于inet_pton
#include <functional>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <sys/socket.h> // 添加这个用于socket函数
#include <unistd.h>     // 添加这个用于close
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>



class VideoClient {
public:
  VideoClient(const std::string &ip, int port);
  ~VideoClient();
  
  using CallBackResult = void;
  using FrameCallBack = std::function<CallBackResult(const cv::Mat&)>;
  void setFrameCallBack(FrameCallBack callback) { framecallback = callback; }
  
  bool connect();
  void run();
  void cleanup();
  
  bool disconnect;
  cv::Mat currentFrame;
  
private:
  static const int BUFFER_SIZE;
  static const int HEADER_SIZE;

  std::string serverIp;
  int port;
  int sock;
  struct sockaddr_in serverAddr;
  std::vector<char> buffer;
  
  FrameCallBack framecallback;

  bool initSocket();
  std::vector<char> receiveFrame();
  cv::Mat decodeFrame(const std::vector<char> &frameData);
};

#endif // VIDEO_STREAM_HPP
