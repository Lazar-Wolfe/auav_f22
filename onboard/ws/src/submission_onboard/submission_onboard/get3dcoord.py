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

class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')
        # self.publisher_ = self.create_publisher(Pose,'car3dcoord',10)
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period,self.publisher_function)
        self.br = CvBridge()
        self.imagex = None
        self.imagey = None
        self.coord3dx = None
        self.coord3dy = None
        self.coord3dz = None
        self.rgbimage = None
        self.x_drone = 0
        self.y_drone = 0
        self.z_drone = 0
        self.orientationx = 0
        self.orientationy = 0
        self.orientationz = 0
        self.orientationw = 0
        self.depth_image = None
        self.subscription = self.create_subscription(Pose, '/detector_topic',self.coord_callback,10)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/depth_image',self.depthimage_callback,10)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/VehicleLocalPosition_PubSubTopic',self.drone_odom_callback,10)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/image',self.rgbimage_callback,10)
        self.subscription = self.create_subscription(VehicleAttitude, '/VehicleAttitude_PubSubTopic', self.drone_orientation_callback, 10)
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
        header_depthimage = data.header
        self.depthimage = cv_depth
        self.projection()
        
    def projection(self):
        if self.imagex == -999 or self.imagey == -999:
            return
        if self.x_drone is not None and self.depthimage is not None and self.imagex is not None and self.imagey is not None and self.rgbimage is not None:
            cx = int(self.imagex)
            cy = int(self.imagey)
            img = self.depthimage
            #TODO: get drone pose

            # cx = cx.reshape(-1)
            # cy = cy.reshape(-1)
            # mask = cx<480
            # mask=mask&(cx>=0)
            # mask=mask&(cy<640)
            # mask=mask&(cy>=0)
            # cx = cx[mask]
            # cy = cy[mask]

            cx=np.int32(cx)
            cy=np.int32(cy)

            X = np.vstack((cx,cy,np.ones(1)))
            #TODO: get K from camera_info
            # k_int = np.array([[]])
            k_int = np.array([[465.60148469763925, 0.0, 320.5],
                                [0.0, 465.60148469763925, 240.5],
                                [0.0, 0.0, 1.0]])
            X=np.matmul(np.linalg.inv(k_int),X)
            
            unit_vec=X/np.linalg.norm(X,axis=0)
            #TODO: depth wrong
            dep=img[cx,cy]
            print("Depth",dep)
            cv2.circle(self.rgbimage,(cx,cy),5,(0,255,0),-1)
            cv2.imshow("image",self.rgbimage)
            cv2.waitKey(1)
            new_vec=unit_vec*dep
            quaternion = (self.orientationx,self.orientationy,self.orientationz,self.orientationw)
            mat = tf.quaternion_matrix(quaternion)
            transform_mat=mat
            transform_mat[:3,3]=[self.x_drone,self.y_drone,self.z_drone]
            new_vec_p = np.vstack((new_vec, np.ones(new_vec.shape[1])))
            transform_mat_c_d = np.array([[-1., 0., 0., 0.09],
                                            [0.,-1., 0., 0.],
                                            [0., 0., -1., -0.01],
                                            [0., 0., 0., 1.]])
            coord = np.matmul(transform_mat_c_d, new_vec_p)
            coord=np.matmul(transform_mat, coord)
            print(coord)
            # print(self.x_drone,self.y_drone,self.z_drone)
            # print(cx,cy)
            self.get_logger().info("Got 3d coordinate")
            # return coord


def main(args=None):
    rclpy.init(args=args)

    node_ = TransformationNode()

    rclpy.spin(node_)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()