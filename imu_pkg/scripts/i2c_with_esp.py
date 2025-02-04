import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math
import serial
import time

serial_port = "/dev/ttyAMA2"
baud_rate = 115200

# 设置写超时，防止写操作长时间阻塞
ser = serial.Serial(port=serial_port, baudrate=baud_rate,
                    timeout=1, write_timeout=0.1)
print(f"打开串口 {serial_port}，波特率 {baud_rate}")


def imu_callback(msg):
    quaternion = [msg.orientation.x,
                  msg.orientation.y,
                  msg.orientation.z,
                  msg.orientation.w]

    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    roll = roll * 180 / math.pi
    pitch = pitch * 180 / math.pi
    yaw = 360 + (yaw * 180 / math.pi) if yaw < 0 else yaw * 180 / math.pi
    rospy.loginfo("Roll=%.0f Pitch=%.0f Yaw=%.0f", roll, pitch, int(yaw))

    # 将数据转换为字符串，并添加换行符以便接收端正确分隔数据
    data = f"{yaw}\n"
    try:
        ser.write(data.encode('utf-8'))
        ser.flush()  # 立即刷新，尽量减少内部缓存
    except serial.SerialTimeoutException:
        rospy.logwarn("串口写入超时！")
    except Exception as e:
        rospy.logerr(f"串口写入错误: {e}")


if __name__ == "__main__":
    rospy.init_node("i2c_with_esp_node")
    imu_sub = rospy.Subscriber("/imu/data", Imu, imu_callback, queue_size=10)
    rospy.spin()
