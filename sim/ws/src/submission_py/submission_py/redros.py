#!/usr/bin/env pyton3
from turtle import down
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import cv2, numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge,CvBridgeError


def red_detector(frame):
     
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,19,85])
    upper_red = np.array([1,255,255])  
    mask = cv2.inRange(hsv, lower_red, upper_red)
 
    mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    _,th = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
    white = np.argwhere(th == 255)
    white = np.transpose(white)
    if white.size != 0:
        
        y = np.mean(white,axis=1)
        cv2.circle(mask_3,(int(y[1]),int(y[0])),2,(0,0,255),3)
        # y[0] is down
        # y[1] is right
        print(y)
    print(mask_3.shape)
    return mask_3
 


class DetectorNode(Node):

    def __init__(self):
        super().__init__('drone_node')
        self.publisher_ = self.create_publisher(Image, 'detector_topic', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.br = CvBridge()
        self.imgimg = None
        self.ans = None
        self.subscription = self.create_subscription(Image,'/rgbd_camera/image',self.listener_callback,10)
        self.subscription

    def listener_callback(self,data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        header_imgimg = data.header
        self.imgimg = cv_image
        self.ans = red_detector(self.imgimg)

    def timer_callback(self):
        if self.ans is not None:

            # img = cv2.resize(self.imgimg,(300,300))
            img = self.ans
            image_message = Image()
            image_message = self.br.cv2_to_imgmsg(img, encoding="bgr8")
            self.publisher_.publish(image_message)
            self.get_logger().info('Publishing new image')
            self.ans = None


def main(args=None):
    rclpy.init(args=args)

    drone_node = DetectorNode()

    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()