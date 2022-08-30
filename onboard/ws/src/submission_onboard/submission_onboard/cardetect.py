#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2, numpy as np
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

# def car_detector(frame,depth):

class DetectorNode(Node):
    def __init__(self):
        super().__init__('contour_detector')
        self.publisher_ = self.create_publisher(Pose,'detector_topic',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.publisher_function)
        
        self.br = CvBridge()
        self.rgbimage = None
        self.depthimage = None

        self.x = None
        self.y = None

        self.subscription = self.create_subscription(Image,'/rgbd_camera/image',self.rgbimage_callback,10)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/depth_image',self.depthimage_callback,10)


    def publisher_function(self):
        if self.x is not None:
            msg = Pose()
            msg.position.y = float(self.y)
            msg.position.x = float(self.x)
            self.publisher_.publish(msg)
            self.get_logger().info("Published car image coordinates")
            self.x =None
            self.y =None
            self.ans =None

    def rgbimage_callback(self,data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.rgbimage = cv_image
        try:
            self.ans,self.x,self.y = self.car_detector()
        except:
            self.x,self.y=-999,-999

    def depthimage_callback(self,data):
        try:
            cv_depth = self.br.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.depthimage = cv_depth

    def car_detector(self):
        print("Detecting car")
        img = self.rgbimage
        img = cv2.resize(img,(640,480))

        gray = cv2.cv2tColor(img,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,9,255,cv2.THRESH_BINARY_INV)

        contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        # print(len(contours))
        min_area=40
        max_area=400*600

        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            area = cv2.contourArea(cnt)
            if rect[1][0]==0 or rect[1][1]==0:
                continue
            # area_ratio = area/(rect[1,0]*rect[1,1])
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)

            if area>min_area and area<max_area:
                cv2.drawContours(img,[approx],0,(0,255,0),2)
            

        # cv2.drawContours(img, contours, -1,(0,0,255),3)
        cv2.imshow("cAR",img)
        cv2.waitKey(100)
        # cv2.imshow("cAR",thresh)
        # cv2.waitKey(100)
        cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)
    car_detector_node = DetectorNode()
    rclpy.spin(car_detector_node)
    car_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()