import message_filters
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import CameraInfo
import numpy as np

def callback(rgb_msg, camera_info):
   rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
   camera_info_K = np.array(camera_info.K).reshape([3, 3])
   camera_info_D = np.array(camera_info.D)
   rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)
   print(rgb_image)
   cv2.imshow('image',rgb_image)
   cv2.waitKey(0)


if __name__ == '__main__':
   rospy.init_node('image_node', anonymous=True)
   #/desistek_saga/desistek_saga/camera/camera_image
   #/desistek_saga/desistek_saga/camera/camera_image/theora/parameter_descriptions
   #/desistek_saga/desistek_saga/camera/camera_info
   # image_sub = message_filters.Subscriber('/desistek_saga/desistek_saga/camera/camera_image', Image)
   # info_sub = message_filters.Subscriber('/desistek_saga/desistek_saga/camera/camera_info', CameraInfo)
   # print(image_sub)


   #/bluerov2/camera_out/image_raw/compressed
   #/bluerov2/camera_out/image_raw/camera_info
   #/bluerov2/camera_out/image_raw/compressedDepth
  
   #/desistek_saga/sonar
   # image_sub = message_filters.Subscriber('/desistek_saga/sonar', Image)

   image_sub = message_filters.Subscriber('/bluerov2/camera_out/image_raw', Image)
   info_sub = message_filters.Subscriber('/bluerov2/camera_out/camera_info', CameraInfo)
   print(image_sub)
   ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
   ts.registerCallback(callback)
   rospy.spin()