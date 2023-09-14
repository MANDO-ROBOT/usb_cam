#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_callback(msg):
    try:
        # 이미지 메시지를 OpenCV 이미지로 변환
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 이미지 일부를 자릅니다. (예: 가운데 50x50 픽셀)
        cropped_image = cv_image[0:480, 0:670]

        # 자른 이미지를 다시 ROS 이미지 메시지로 변환
        cropped_image_msg = bridge.cv2_to_imgmsg(cropped_image, "bgr8")

        # 수정된 이미지를 발행
        cropped_image_pub.publish(cropped_image_msg)
    except CvBridgeError as e:
        rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('image_cropping_node')
    
    # 입력 이미지 토픽 및 출력 이미지 토픽 설정
    input_image_topic = "/zed2_stereo/image_raw"  # 입력 이미지 토픽
    output_image_topic = "/image_cropped"   # 수정된 이미지 토픽
    
    # 이미지 토픽을 구독하고 콜백 함수를 등록
    rospy.Subscriber(input_image_topic, Image, image_callback)
    
    # 수정된 이미지를 발행할 토픽 생성
    cropped_image_pub = rospy.Publisher(output_image_topic, Image, queue_size=10)
    
    rospy.spin()
