#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"

void IMUCallback(sensor_msgs::Imu msg) {
  tf::Quaternion quaternion(msg.orientation.x, msg.orientation.y,
                            msg.orientation.z, msg.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  roll = roll * 180 / M_PI;
  yaw = yaw * 180 / M_PI;
  pitch = pitch * 180 / M_PI;

  ROS_INFO("Roll=%.0f Pitch=%.0f Yaw=%.0f", roll, pitch, yaw);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "imu_node");

  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("/imu/data", 10, IMUCallback);

  ros::spin();

  return 0;
}
