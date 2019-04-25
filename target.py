#!/usr/bin/env python

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class convert_and_target:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("xtion/rgb/image_raw", Image, self.callback)
    
    def callback(self, data):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (290,310), 25, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        

if __name__ == '__main__':
    
    cat = convert_and_target()
    rospy.init_node('convert_and_target', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
