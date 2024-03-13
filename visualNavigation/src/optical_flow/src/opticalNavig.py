import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mavros_msgs.msg import OpticalFlowRad
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu


class OpticalFlowCalculator():
    def __init__(self):
        self.IrisCameraSubscriber = rospy.Subscriber("/iris/camera1/image_raw",Image,self.IrisCameraCallback)
        self.LidarSubscriber = rospy.Subscriber("/mavros/distance_sensor/range_down",Range,self.LidarSubscriberCallBack)
        self.FlowPublisher = rospy.Publisher("/mavros/px4flow/raw/send",OpticalFlowRad,queue_size=10)
        self.CurrentFrame = None
        self.PreviousFrame = None
        self.bridge = CvBridge()
        self.flow = OpticalFlowRad
        self.fov = 1.3962634
        self.height = 800
        self.width = 800
        self.rangeDown = None
        self.lidarDeltaTime = None
        self.lastLidarTime = None
        self.lastTimeFlow = 0
        self.currTimeFlow = None

    def LidarSubscriberCallBack(self,msg):
        self.rangeDown = msg.range
        currTime =int((msg.header.stamp.secs*1e9+msg.header.stamp.nsecs)*1e-3)
        if self.lastLidarTime is None:
            self.lastLidarTime = currTime
        self.lidarDeltaTime = int(currTime - self.lastLidarTime)
        self.lastLidarTime = currTime

    def IrisCameraCallback(self,msg):
        if self.CurrentFrame is None:
            self.CurrentFrame = self.bridge.imgmsg_to_cv2(msg)
            self.CurrentFrame = cv2.cvtColor(self.CurrentFrame,cv2.COLOR_BGR2GRAY)  
            self.CurrentFrame = self.CurrentFrame.astype("float32") 
            self.PreviousFrame = self.CurrentFrame.copy()
            self.currTimeFlow = (msg.header.stamp.secs*1e9 +msg.header.stamp.nsecs)*1e-3
            self.lastTimeFlow = self.currTimeFlow
        else: 
            self.PreviousFrame = self.CurrentFrame.copy()
            self.CurrentFrame = self.bridge.imgmsg_to_cv2(msg)
            self.CurrentFrame = cv2.cvtColor(self.CurrentFrame,cv2.COLOR_BGR2GRAY)
            self.CurrentFrame = self.CurrentFrame.astype("float32")
            self.lastTimeFlow = self.currTimeFlow
            self.currTimeFlow = (msg.header.stamp.secs*1e9 +msg.header.stamp.nsecs)*1e-3
        self.calculateFlow()
        
    def calculateFlow(self):
        flow,qual = cv2.phaseCorrelate(self.PreviousFrame,self.CurrentFrame)

        dx = flow[0]
        dy = flow[1]

        shiftxRad = float((dx/self.width)*self.fov)
        shiftyRad = float((dy/self.height)*self.fov)

        flow = OpticalFlowRad()
        flow.integrated_x = -shiftyRad
        flow.integrated_y = -shiftxRad
        flow.distance = self.rangeDown

        flow.quality = 200
        flow.header.stamp.secs = rospy.Time.now().secs
        flow.header.stamp.nsecs = rospy.Time.now().nsecs


        
        dt = self.currTimeFlow - self.lastTimeFlow
        flow.integration_time_us = int(dt)
        flow.time_delta_distance_us = int(self.lidarDeltaTime)

        self.FlowPublisher.publish(flow)


        
            



if __name__ == "__main__":
    rospy.init_node("Optical_Flow_Calculator",anonymous=True)
    flowCalculator = OpticalFlowCalculator()
    rospy.spin()