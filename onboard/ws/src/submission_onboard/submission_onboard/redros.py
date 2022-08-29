#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import cv2, numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge,CvBridgeError
# import tf
import geometry_msgs
from geometry_msgs.msg import Pose
# import tf.transformations
# from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_matrix



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
        # print(mask_3.shape)
        return mask_3,int(y[1]),int(y[0])
 


class DetectorNode(Node):

    def __init__(self):
        super().__init__('drone_node')

        # self.publisher_ = self.create_publisher(Image, 'detector_topic', 10)
        self.publisher_ = self.create_publisher(Pose,'detector_topic',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publisher_function)

        self.br = CvBridge()
        self.rgbimage = None
        self.depthimage = None
        self.ans = None
        self.x = None
        self.y = None

        # Subscribers
        self.subscription = self.create_subscription(Image,'/rgbd_camera/image',self.rgbimage_callback,10)
        # self.subscription = self.create_subscription(Image,'/rgbd_camera/depth_image',self.depthimage_callback,10)
        # self.subscription = self.create_subscription(P,'/odom',self.odom_callback,10)

    def rgbimage_callback(self,data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        header_rgbimage = data.header
        self.rgbimage = cv_image
        try:
            self.ans,self.x,self.y = red_detector(self.rgbimage)
        except:
            self.x,self.y=-999,-999

    # def depthimage_callback(self,data):
    #     try:
    #         cv_depth = self.br.imgmsg_to_cv2(data, "passthrough")
    #     except CvBridgeError as e:
    #         print(e)
    #     header_depthimage = data.header
    #     self.depthimage = cv_depth
        # self.car_pose = projection(self.x,self.y,self.self.depthimage)

        

    def publisher_function(self):
        if self.x is not None:
            # img = cv2.resize(self.rgbimage,(300,300))
            # if self.depthimage is not None:
            #     img = self.ans
            #     image_message = Image()
            #     image_message = self.br.cv2_to_imgmsg(img, encoding="bgr8")
            #     self.publisher_.publish(image_message)
            #     self.get_logger().info('Publishing new image')
            #     self.car_pose=projection()
            #     self.ans = None
            msg = Pose()
            msg.position.y = float(self.y)
            msg.position.x = float(self.x)
            self.publisher_.publish(msg)
            self.get_logger().info("Published car image coordinates")
            self.x =None
            self.y =None
            self.ans =None




def main(args=None):
    rclpy.init(args=args)

    drone_node = DetectorNode()

    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()