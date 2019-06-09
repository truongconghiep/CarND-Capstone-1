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
        result = TrafficLight.UNKNOWN
        HsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        red1 = cv2.inRange(HsvImg, np.array([0,50,50]) , np.array([10,255,255]))
        red2 = cv2.inRange(HsvImg, np.array([170,50,50]) , np.array([180,255,255]))
        converted_img_red = cv2.addWeighted(red1, 1.0, red2, 1.0, 0.0)
        blur_img = cv2.GaussianBlur(converted_img_red,(15,15),0)
        circles = cv2.HoughCircles(blur_img,cv2.HOUGH_GRADIENT,0.5,41, param1=70,param2=30,minRadius=5,maxRadius=150)

        yellow1 = cv2.inRange(HsvImg, np.array([22, 100, 160]) , np.array([34, 255, 255]))
        yellow2 = cv2.inRange(HsvImg, np.array([5, 50, 50]) , np.array([15, 255, 255]))
        converted_img_yellow = cv2.addWeighted(yellow1, 1.0, yellow2, 1.0, 0.0)
        blur_img_yellow = cv2.GaussianBlur(converted_img_yellow,(15,15),0)
        circles_yellow = cv2.HoughCircles(blur_img_yellow,cv2.HOUGH_GRADIENT,0.5,41, param1=70,param2=30,minRadius=5,maxRadius=150)

        Green1 = cv2.inRange(HsvImg, np.array([30, 59, 50]) , np.array([79, 158, 171]))
        Green2 = cv2.inRange(HsvImg, np.array([0, 255, 0]) , np.array([229, 255, 204]))
        converted_img_green = cv2.addWeighted(Green1, 1.0, Green2, 1.0, 0.0)
        blur_img_green = cv2.GaussianBlur(converted_img_green,(15,15),0)
        circles_green = cv2.HoughCircles(blur_img_green,cv2.HOUGH_GRADIENT,0.5,41, param1=70,param2=30,minRadius=5,maxRadius=150)
        
        if circles is not None:
            return TrafficLight.RED
        if circles_yellow is not None:
            return TrafficLight.YELLOW
        if circles_green is not None:
            return TrafficLight.GREEN
            #rospy.logwarn("state Yellow {0}".format(0))

        return result
