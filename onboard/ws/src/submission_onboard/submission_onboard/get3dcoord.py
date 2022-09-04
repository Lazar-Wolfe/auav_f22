#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2, numpy as np
# import tf
# from tf.transformations import quaternion_matrix
import tf_transformations as tf
import geometry_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Pose
from px4_msgs.msg import VehicleLocalPosition,VehicleAttitude

# from sensor_msgs.msg import PointCloud2
# from scipy.spatial.transform import Rotation as R
# import tf2_ros
# import tf2_geometry_msgs

previousx = [0.,0.,0.]
previousy = [0.,0.,0.]
class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')
        self.publisher_ = self.create_publisher(Pose,'Car_pose',100)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.publisher_function)
        self.br = CvBridge()
        self.imagex = None
        self.imagey = None
        self.coord3dx = None
        self.coord3dy = None
        self.rgbimage = None
        self.x_drone = 0
        self.y_drone = 0
        self.z_drone = 0
        self.orientationx = 0
        self.orientationy = 0
        self.orientationz = 0
        self.orientationw = 0
        self.depthimage = None
        self.subscription = self.create_subscription(Pose, '/detector_topic',self.coord_callback,100)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/depth_image',self.depthimage_callback,100)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/VehicleLocalPosition_PubSubTopic',self.drone_odom_callback,100)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/image',self.rgbimage_callback,100)
        self.subscription = self.create_subscription(VehicleAttitude, '/VehicleAttitude_PubSubTopic', self.drone_orientation_callback, 100)
    def coord_callback(self,data):
        self.imagex = data.position.x
        self.imagey = data.position.y

    def drone_odom_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.x_drone = msg.y
            self.y_drone = msg.x
            self.z_drone = -msg.z
            
    def drone_orientation_callback(self, msg):
        self.orientationw = msg.q[0]
        self.orientationx = msg.q[1]
        self.orientationy = msg.q[2]
        self.orientationz = msg.q[3]
    def rgbimage_callback(self,data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        header_rgbimage = data.header
        self.rgbimage = cv_image
    def depthimage_callback(self,data):
        try:
            cv_depth = self.br.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.depthimage = cv_depth
        self.projection()
    def publisher_function(self):
        if self.coord3dx is not None and self.coord3dy is not None:
            msg = Pose()
            msg.position.x = float(self.coord3dx)
            msg.position.y = float(self.coord3dy)
            msg.position.z = float(0)
            self.publisher_.publish(msg)
            # self.get_logger().info("Published car world coordinates")
            self.coord3dx = self.coord3dy = None

    def projection(self):
        if self.imagex == -999 or self.imagey == -999:
            msg = Pose()
            msg.position.x = float(previousx[-1])
            msg.position.y = float(previousy[-1]) 
            msg.position.z = float(0)
            self.publisher_.publish(msg)
            return
        if self.x_drone is not None and self.depthimage is not None and self.imagex is not None and self.imagey is not None and self.rgbimage is not None:
            cx = int(self.imagex)
            cy = int(self.imagey)
            k_int = np.array([[465.60148469763925,0,320.5],
                              [0,465.60148469763925,240.5],
                              [0,0,1]])
            pleasex = (cx-320)*self.depthimage[cy,cx]/465.60148469763925
            pleasey = (cy-240)*self.depthimage[cy,cx]/465.60148469763925
            pleasez = self.depthimage[cy,cx]
            # print(f"Image X: {self.imagex}, Y: {self.imagey}")
            # print(f"Depth X:{cx}, Y:{cy}")
            # print(f"Depth {self.depthimage[cy,cx]}")

            
            # Coordinates with respect to camera frame
            # print(f"Please X: {pleasex}, Y: {pleasey}, Z: {pleasez}") 
            # Correct till here

            cv2.circle(self.rgbimage,(int(cx),int(cy)),5,(0,255,0),-1)
            X = np.array([pleasex,pleasey,pleasez])
            np.reshape(X,((3,1)))
            transform_camera_to_drone = np.array([[1.,0.,0.],
                                                  [0.,0.,1.],
                                                  [0.,-1.,0.]])
            # print(X.shape)
            X = np.matmul(transform_camera_to_drone,X)


            # Coordinates with respect to drone frame
            # print(f"X: {X[0]}, Y: {X[1]}, Z: {X[2]}")
            # Correct till here

            quaternion = (self.orientationx,self.orientationy,self.orientationz,self.orientationw)
            euler = tf.euler_from_quaternion(quaternion)
            print(f"Roll: {euler[0]}, Pitch: {euler[1]}, Yaw: {euler[2]}")
            mat = tf.quaternion_matrix(quaternion)
            transform_mat = mat
            # print(f"drone x {self.x_drone} y {self.y_drone} z {self.z_drone}")
            transform_mat[:3,3] = [self.x_drone,self.y_drone,self.z_drone]
            # print(f"Drone X: {self.x_drone}, Y: {self.y_drone}, Z: {self.z_drone}")
            # print(f"Transform Matrix: {transform_mat}")
            X = np.append(X,1)
            # print(X)
            # print(X.shape)
            X = np.matmul(X,transform_mat)
            print(f"World X: {X[0]}, Y: {X[1]}, Z: {X[2]}")

            # print(f"X: {X[0]}, Y: {X[1]}, Z: {X[2]}")
            # print(transform_mat.shape)
            # cv2.circle(self.depthimage,(self.imagex,self.imagey),5,(0,255,0),-1)
            cv2.imshow("RGB",self.rgbimage)
            # cv2.imshow("Depth",self.depthimage)
            cv2.waitKey(1)
            # print(f"Drone X: {self.x_drone}, Y: {self.y_drone}, Z: {self.z_drone}")
            # print(f"Actual X {self.x_drone-pleasex}, Y {self.y_drone+pleasey}")
            
def main(args=None):
    rclpy.init(args=args)

    node_ = TransformationNode()

    rclpy.spin(node_)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()