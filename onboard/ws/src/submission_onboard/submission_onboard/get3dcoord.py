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
        timer_period = 0.01
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
        self.orientationx = msg.q[0]
        self.orientationy = msg.q[1]
        self.orientationz = msg.q[2]
        self.orientationw = msg.q[3]
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
            return
        if self.x_drone is not None and self.depthimage is not None and self.imagex is not None and self.imagey is not None and self.rgbimage is not None:
            cx = int(self.imagex)
            cy = int(self.imagey)
            k_int = np.array([[465.60148469763925,0,320.5],
                              [0,659.4171771583216,240.5],
                              [0,0,1]])
            X = np.array([[cx],[cy],[1]])
            X = np.matmul(np.linalg.inv(k_int),X)
            unit_vec = X/np.linalg.norm(X,axis=0)
            dep = self.depthimage[cy,cx]
            new_vec = dep*unit_vec
            euler = tf.euler_from_quaternion([self.orientationx,self.orientationy,self.orientationz,self.orientationw])
            quaternion = (self.orientationx,self.orientationy,self.orientationz,self.orientationw)
            mat = tf.quaternion_matrix(quaternion)
            transform_mat = mat
            transform_mat[:3,3] = [self.y_drone,self.x_drone,self.z_drone]
            new_vec_p = np. vstack((new_vec,np.ones(new_vec.shape[1])))
            transform_mat_c_d = np.array([[1.,0.,0.,0.],
                                          [0.,0.,1.,0.],
                                          [0.,-1.,0.,0.],
                                          [0.,0.,0.,1.]])
            coord = np.matmul(transform_mat_c_d,new_vec_p)
            coord = np.matmul(transform_mat,coord)
            self.coord3dx = coord[0]
            self.coord3dy = -coord[1]+0.2
            previousx.append(self.coord3dx)
            previousy.append(self.coord3dy)
            cond = abs(previousx[-1]-previousx[-2])+abs(previousy[-1]-previousy[-2])
            if(len(previousx)>10):
                previousx.pop()
                previousy.pop()
            if not cond:
                previousx.pop(0)
                previousy.pop(0)
            if cond<2:
                print(f"X = {self.coord3dx}\t Y = {self.coord3dy}")
                msg = Pose()
                msg.position.x = float(self.coord3dx)
                msg.position.y = float(self.coord3dy)
                msg.position.z = float(0)
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node_ = TransformationNode()

    rclpy.spin(node_)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()