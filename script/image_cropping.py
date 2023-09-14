#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

def image_callback(msg):
    try:
        # Convert the image message to an OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Crop a portion of the image (e.g., a 50x50 pixel center)
        cropped_image = cv_image[0:480, 0:675]

        # Compress the cropped image as JPEG
        _, compressed_image_data = cv2.imencode(".jpg", cropped_image)

        # Create a CompressedImage message and set its data field
        compressed_image_msg = CompressedImage()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = compressed_image_data.tobytes()

        # Publish the compressed image
        cropped_image_pub.publish(compressed_image_msg)
    except CvBridgeError as e:
        rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('image_cropping_node')
    
    # Set the input and output image topics
    input_image_topic = "/zed2_stereo/image_raw"  # Input image topic
    output_image_topic = "/wm_robot/compressed_image"   # Modified compressed image topic
    
    # Subscribe to the image topic and register the callback function
    rospy.Subscriber(input_image_topic, Image, image_callback)
    
    # Create a topic to publish the modified compressed image
    cropped_image_pub = rospy.Publisher(output_image_topic, CompressedImage, queue_size=10)
    
    rospy.spin()
