# -*- coding: utf-8 -*-

import numpy
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2


def run():
    rospy.init_node("cv_test_node",anonymous=True)
    capture = cv2.VideoCapture("/home/chen/视频/屏幕录像 2021-12-14 23:38:06.mp4")
    imgPub = rospy.Publisher("/cv_test/image",Image,queue_size=5)
    rospy.loginfo("raft node publishing")
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ref,frame = capture.read()
        image = Image()
        image.encoding = 'bgr8'
        image.height = frame.shape[0]
        image.width = frame.shape[1]
        image.step = frame.shape[1] * frame.shape[2]
        image.data = numpy.array(frame).tostring()
        image.header.stamp = rospy.Time.now()
        imgPub.publish(image)
        rate.sleep()



if __name__=='__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass




