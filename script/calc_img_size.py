#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import sys
from io import BytesIO

class ImageDataSizeCalculator:
    def __init__(self):
        rospy.init_node('image_data_size_calculator', anonymous=True)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_callback)
        self.total_data_size = 0
        self.duration = rospy.Duration(1)  # 1 second
        self.timer = rospy.Timer(self.duration, self.calculate_total_size)

    def image_callback(self, msg):
        data_size = sys.getsizeof(msg.data)  # Calculate data size in bytes
        buff = BytesIO()
        msg.serialize(buff)
        serialized_bytes = buff.getvalue()
        print(len(serialized_bytes))
        self.total_data_size += data_size

    def calculate_total_size(self, event):
        # rospy.loginfo("Total data size received on the image topic in 1 seconds: %d bytes" % self.total_data_size)
        self.total_data_size = 0

def main():
    try:
        calculator = ImageDataSizeCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
