#!/usr/bin/env python3
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf_transformations as tf
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from px4_msgs.msg import TrajectorySetpoint



class worldToImage(Node):
    def __init__(self):
        super().__init__('world2image_node')
        # publish depth of real world coodinates
        self.publisher_ = self.create_publisher(Float32,'distance_to_collision',100)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.publisher_function)
        
        self.br = CvBridge()
        self.imagex = self.imagey = None
        self.coord3dx = self.coord3dy = self.coord3dz = None
        self.dronex = self.droney = self.dronez = None
        self.orientationx = self.orientationy = self.orientationz = self.orientaionw = None
        self.depthimage = None
        self.rgbimage  = None
        
        self.subscription = self.create_subscription(VehicleLocalPosition,'/VehicleLocalPosition_PubSubTopic',self.drone_odom_callback,100)
        self.subscription = self.create_subscription(VehicleAttitude, '/VehicleAttitude_PubSubTopic', self.drone_orientation_callback, 100)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/depth_image',self.depthimage_callback,100)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/image',self.rgbimage_callback,100)
        self.subscription = self.create_subscription(TrajectorySetpoint,'/TrajectorySetpoint_PubSubTopic',self.trajectory_callback,100)

    def drone_odom_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.dronex = msg.y
            self.droney = msg.x
            self.dronez = -msg.z
            
    def drone_orientation_callback(self, msg):
        self.orientationw = msg.q[0]
        self.orientationx = msg.q[1]
        self.orientationy = msg.q[2]
        self.orientationz = msg.q[3]

    def trajectory_callback(self,data):
        msg = TrajectorySetpoint()
        self.coord3dx = data.y
        self.coord3dy = data.x
        self.coord3dz = -data.z

    def rgbimage_callback(self,data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.rgbimage = cv_image
        
    def depthimage_callback(self,data):
        try:
            cv_depth = self.br.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.depthimage = cv_depth

    def transform(self):
        k_int = np.array([[465.60148469763925,0,320.5],
                              [0,465.60148469763925,240.5],
                              [0,0,1]]) 
        

def main(args=None):
    rclpy.init(args=args)

    node_ = worldToImage()

    rclpy.spin(node_)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()