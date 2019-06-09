from styx_msgs.msg import TrafficLight
import cv2
#import rospy
import numpy as np
#from std_msgs.msg import Int32

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        HsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        frame_threshed = cv2.inRange(HsvImg, np.array([0, 120, 120],np.uint8), np.array([10, 255, 255],np.uint8))
        r = cv2.countNonZero(frame_threshed)
        if r > 50:
            return TrafficLight.RED

        frame_threshed = cv2.inRange(HsvImg, np.array([28, 120, 120],np.uint8), np.array([43, 255, 255],np.uint8))
        y = cv2.countNonZero(frame_threshed)
        if y > 50:
            return TrafficLight.YELLOW

        frame_threshed = cv2.inRange(HsvImg, np.array([64, 120, 120],np.uint8), np.array([99, 255, 255],np.uint8))
        g = cv2.countNonZero(frame_threshed)
        if g > 50:
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
