#!/usr/bin/env python

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class convert_and_target:

    def __init__(self):

        self.bridge = CvBridge()
        print("Setup Subscriber")
#        self.image_sub = rospy.Subscriber("xtion/rgb/image_raw", Image, self.callback)
#        self.image_sub = rospy.Subscriber("xtion/rgb/image_raw/compressed", CompressedImage, self.callback)

		#If having issues with bandwidth run the following command on tiago to create a throttled topic:
		# rosrun topic_tools drop /xtion/rgb/image_raw/compressed 9 10 xtion/throttled/compressed
        self.image_sub = rospy.Subscriber("xtion/throttled/compressed", CompressedImage, self.callback)
    
    def callback(self, msg):
#        print("Image received")

        if isinstance(msg, CompressedImage):
            # Decompress message.
            msg = self.from_raw(msg.data)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)
            
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (290,295), 25, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        
    def from_raw(self, raw, compress=False):
        """Deserializes binary-encoded image data into a ROS Image message.

        Args:
            raw: Binary encoded image data.
            compress: Whether to return a compressed image or not.

        Returns:
            ROS Image or CompressedImage message.

        Raises:
            CvBridgeError: On image conversion error.
        """
        # Convert to OpenCV image.
        nparr = numpy.fromstring(raw, numpy.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Convert to ROS message.
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(img)

        return msg 
        

if __name__ == '__main__':
    
    cat = convert_and_target()
    rospy.init_node('convert_and_target', anonymous=True)
    print("Init finished")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
