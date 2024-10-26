#!/usr/bin/env python

import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class Listener:
    def __init__(self) -> None:
        self.rospy = rospy
        self.bridge = CvBridge()
        """ subscribe rbg/depth image simultaneously"""
        self.img_pub = rospy.Publisher("/image", Image, queue_size=1)
        
    def pub_img(self, rgb_img,header):
        out_msg = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8", header)
        
        self.img_pub.publish(out_msg)
    
    def __del__(self):
        print("__del__")
        


if __name__ == "__main__":
    rospy.init_node("ros_tester", anonymous=True)
    listener = Listener()
    rospy.sleep(0.5)
    i=0
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        i=(i+1)%1000
        if i>=0:
            img = np.ones((480, 640, 3), dtype=np.uint8)
            img[np.arange(480), :,:] =(np.arange(640)%i)[:,None]
        else:
            img = np.ones((480, 1280, 3), dtype=np.uint8)
            
        cv2.putText(img, f"Hello World {i}", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        listener.pub_img(img, header)
        rospy.loginfo(f"pub {i}, {header.stamp.to_sec()}")
        r.sleep()

    rospy.spin()