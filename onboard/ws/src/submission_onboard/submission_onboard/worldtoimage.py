#!/usr/bin/env python3
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf_transformations as tf
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from px4_msgs.msg import TrajectorySetpoint,VehicleLocalPosition,VehicleAttitude



class worldToImage(Node):
    def __init__(self):
        super().__init__('world2image_node')
        # publish depth of real world coodinates
        self.publisher_ = self.create_publisher(Float32,'distance_to_collision',100)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.publisher_function)
        
        self.br = CvBridge()
        self.imagex = self.imagey = None
        self.dist_to_collision = None
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

    def publisher_function(self):
        if self.dist_to_collision is not None:
            msg = Float32()
            msg.data = self.dist_to_collision
            self.publisher_.publish(msg)
            print(f"Published distance to collision {msg.data}")
            self.dist_to_collision = None
    def trajectory_callback(self,data):
        msg = TrajectorySetpoint()
        self.coord3dx = data.y
        self.coord3dy = data.x
        self.coord3dz = -data.z
        # self.coord3dx =-2.0
        # self.coord3dy = 4.0
        # self.coord3dz = 2.25
        self.transform()

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
        # print("Just inside transform")
        if self.dronez is not None and self.coord3dx is not None and self.rgbimage is not None:
            # print(f"COORD {self.coord3dx,self.coord3dy,self.coord3dz}")
            # print(f"DRONE {self.dronex,self.droney,self.dronez}")
            self.coord3dx -= self.dronex
            self.coord3dy -= self.droney
            self.coord3dz -= self.dronez
            # print("HERE")
            # print(self.coord3dx,self.coord3dy,self.coord3dz)
            quaternion = (self.orientationx,self.orientationy,self.orientationz,self.orientationw)
            euler = tf.euler_from_quaternion(quaternion)
            # print(f"Roll: {euler[0]}, Pitch: {euler[1]}, Yaw: {euler[2]}")
            mat = tf.quaternion_matrix(quaternion)
            transform_mat = mat
            # print(f"drone x {self.x_drone} y {self.y_drone} z {self.z_drone}")
            transform_mat[:3,3] = [self.dronex,self.droney,self.dronez]
            transform_mat = np.linalg.inv(transform_mat)
            # print(f"TRNASFORM MAT {transform_mat}")
            X = np.array([self.coord3dx,self.coord3dy,self.coord3dz,1])
            X = np.matmul(X,transform_mat)
            # where drone wants to go wrt drone frame
            # print("FFFFFFF")
            # temp = X[1]
            # X[1] = X[0]
            # X[0] = temp
            # print(f"WRT DRONE{X}")
            transform_camera_to_drone = np.array([[1.,0.,0.],
                                                  [0.,0.,1.],
                                                  [0.,-1.,0.]])
            # print(f"X: {X}")
            X = np.array([X[0],X[1],X[2]])

            # print(f"Transfprm {transform_camera_to_drone.shape}, X {X.shape}")
            transform_camera_to_drone = np.linalg.inv(transform_camera_to_drone)
            X = np.matmul(transform_camera_to_drone,X)
            # X = -X
            # X[0]=-X[0]
            # X[2]=-X[2]
            # print(f"In camera frame drone should go to: {X}")
            depth = X[2]
            # print(f"DEPTH {depth}")
            self.dist_to_collision = depth
            
            # self.imagex = ((X[1]*465.60148469763925)/depth)+320
            # self.imagey = ((X[0]*465.60148469763925)/depth)+240
            # print(f"FROM DEPTH IMAGE {self.depthimage[int(self.imagey),int(self.imagex)]}")
            # print(f"FROM DEPTH IMAGE {self.depthimage[int(self.imagex),int(self.imagey)]}")
            # print(f"Center{int(self.imagey),int(self.imagex)}")
            # try:
            #     cv2.circle(self.rgbimage,(int(self.imagey),int(self.imagex)),5,(100,200,255),-1)
            #     # print("image coordinates",self.imagex,self.imagey)
            #     cv2.imshow("Image with circle",self.rgbimage)
            #     cv2.waitKey(1)
            # except:
            #     pass
        

def main(args=None):
    rclpy.init(args=args)

    node_ = worldToImage()

    rclpy.spin(node_)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()