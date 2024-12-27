#include <bits/stdint-intn.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sstream> // for converting the command line parameter to integer
#include <thread>

#include "videostream.hpp"

VideoClient::CallBackResult processFrame(const cv::Mat &frame) {
  cv::imshow("result", frame);
}

int main(int argc, char **argv) {

  VideoClient stream("0.0.0.0", 8000);
  stream.setFrameCallBack(processFrame);

  if (stream.connect()) {
    // set stream keeping read data after connection
    stream.disconnect = false;
  } else {
    return 1;
  }

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  cv::Mat image;

  ros::Rate loop_rate(60);

  while (nh.ok()) {

    stream.run();
    image = stream.currentFrame;
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    // cv::imshow("main", frame);
    // cv::waitKey(1);
    loop_rate.sleep();
  }

  return 0;
}
