#include "videostream.hpp"
#include <cstring>
#include <iostream>
#include <map>

const int VideoClient::BUFFER_SIZE = 65507; // UDP最大包大小
const int VideoClient::HEADER_SIZE = 8; // 4字节帧大小 + 4字节包数量

VideoClient::VideoClient(const std::string &ip, int port)
    : serverIp(ip), port(port), sock(-1) {
  buffer.reserve(BUFFER_SIZE);
}

VideoClient::~VideoClient() { cleanup(); }

bool VideoClient::initSocket() {
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
    return false;
  }

  // 设置接收超时
  struct timeval tv;
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    std::cerr << "Error setting timeout: " << strerror(errno) << std::endl;
    return false;
  }

  // 设置接收缓冲区大小
  int rcvbuf = 1024 * 1024; // 1MB
  if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
    std::cerr << "Error setting receive buffer: " << strerror(errno)
              << std::endl;
    return false;
  }

  return true;
}

bool VideoClient::connect() {
  if (!initSocket()) {
    return false;
  }

  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(port);

  if (inet_pton(AF_INET, serverIp.c_str(), &serverAddr.sin_addr) <= 0) {
    std::cerr << "Invalid address: " << strerror(errno) << std::endl;
    return false;
  }

  std::cout << "--------UDP client ready (press q to exit)--------"
            << std::endl;
  return true;
}

std::vector<char> VideoClient::receiveFrame() {
  // 发送帧请求
  const char *request = "REQUEST_FRAME";
  sendto(sock, request, strlen(request), 0, (struct sockaddr *)&serverAddr,
         sizeof(serverAddr));

  // 接收帧头信息
  struct {
    uint32_t frameSize;
    uint32_t totalPackets;
  } header;

  socklen_t serverLen = sizeof(serverAddr);
  int received = recvfrom(sock, &header, sizeof(header), 0,
                          (struct sockaddr *)&serverAddr, &serverLen);

  if (received != sizeof(header)) {
    std::cerr << "Error receiving header" << std::endl;
    return std::vector<char>();
  }

  // 转换网络字节序
  header.frameSize = ntohl(header.frameSize);
  header.totalPackets = ntohl(header.totalPackets);

  // 准备接收所有数据包
  std::map<uint32_t, std::vector<char>> packets;
  std::vector<char> frameData;
  frameData.reserve(header.frameSize);

  // 接收所有数据包
  for (uint32_t i = 0; i < header.totalPackets; ++i) {
    buffer.resize(BUFFER_SIZE);
    received = recvfrom(sock, buffer.data(), BUFFER_SIZE, 0,
                        (struct sockaddr *)&serverAddr, &serverLen);

    if (received <= 4) {
      std::cerr << "Error receiving packet" << std::endl;
      continue;
    }

    // 提取包序号和数据
    uint32_t packetIndex = ntohl(*(uint32_t *)buffer.data());
    std::vector<char> packetData(buffer.begin() + 4, buffer.begin() + received);
    packets[packetIndex] = packetData;
  }

  // 按顺序组装数据包
  for (const auto &packet : packets) {
    frameData.insert(frameData.end(), packet.second.begin(),
                     packet.second.end());
  }

  if (frameData.size() != header.frameSize) {
    std::cerr << "Incomplete frame received" << std::endl;
    return std::vector<char>();
  }

  return frameData;
}

cv::Mat VideoClient::decodeFrame(const std::vector<char> &frameData) {
  try {
    std::vector<uchar> data(frameData.begin(), frameData.end());
    cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR);
    if (frame.empty()) {
      std::cerr << "Failed to decode frame" << std::endl;
    }
    return frame;
  } catch (const cv::Exception &e) {
    std::cerr << "OpenCV error: " << e.what() << std::endl;
    return cv::Mat();
  }
}

void VideoClient::run() {

  auto frameData = receiveFrame();
  // if (frameData.empty())
  // continue;

  currentFrame = decodeFrame(frameData);
  // if (frame.empty())
  // continue;

  if (framecallback && !currentFrame.empty()) {
    // framecallback(frame);
    // cv::waitKey(1);
    // std::cout << "Hello" << std::endl;
  }

  // if (cv::waitKey(1) == 'q')
  // break;

  if (disconnect) {
    cleanup();
  }
}

void VideoClient::cleanup() {
  if (sock >= 0) {
    close(sock);
    sock = -1;
  }
  cv::destroyAllWindows();
}
