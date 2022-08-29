#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2, numpy as np
import tf
import geometry_msgs
from geometry_msgs.msg import Pose
# import tf.transformations
# from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_matrix
# from px4_msgs.msg import 

# def projection(cx,cy,img,K=None):
#     cx = cx.reshape(-1)
#     cy = cy.reshape(-1)
#     mask = cx<480
#     mask=mask&(cx>=0)
#     mask=mask&(cy<640)
#     mask=mask&(cy>=0)
#     cx = cx[mask]
#     cy = cy[mask]

#     cx=np.int32(cx)
#     cy=np.int32(cy)

#     X = np.vstack((cx,cy,np.ones(len(cx))))
#     #TODO: get K from camera_info
#     # k_int = np.array([[]])
#     k_int = np.array([[465.60148469763925, 0.0, 320.5],
#                         [0.0, 465.60148469763925, 240.5],
#                         [0.0, 0.0, 1.0]])
#     X=np.matmul(np.linalg.inv(k_int),X)
    
#     unit_vec=X/np.linalg.norm(X,axis=0)
#     dep=img[cx,cy]
#     new_vec=unit_vec*dep
#     global drone_pose
#     quaternion = (drone_pose.pose.orientation.x,drone_pose.pose.orientation.y,drone_pose.pose.orientation.z,drone_pose.pose.orientation.w)
#     mat = tf.transformations.quaternion_matrix(quaternion)
#     transform_mat=mat
#     transform_mat[:3,3]=[drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
#     new_vec_p = np.vstack((new_vec, np.ones(new_vec.shape[1])))
#     transform_mat_c_d = np.array([[0., -1, 0., 0.],
#                                     [-1., 0., 0., 0.],
#                                     [0., 0., -1., 0.],
#                                     [0., 0., 0., 1.]])
#     coord = np.matmul(transform_mat_c_d, new_vec_p)
#     coord=np.matmul(transform_mat, coord)
#     return coord


class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')
        self.publisher_ = self.create_publisher(Pose,'car3dcoord',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.publisher_function)

        self.image.x = None
        self.image.y = None
        self.coord3d.x = None
        self.coord3d.y = None
        self.coord3d.z = None
        self.drone_pose = None
        sefl.depth_image = None
        self.subscription = self.create_subscription(Pose, '/detector_topic',self.coord_callback,10)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/depth_image',self.depthimage_callback,10)
        self.subscription = self.create_subscription(msg_type, topic, self.drone_pose_callback, 10)
    
    def coord_callback(self,data):
        self.image.x = data.point.x
        self.image.y = data.point.y

    def drone_pose_callback(self,data):
        print("Drone pose callback")

    def depthimage_callback(self,data):
        try:
            cv_depth = self.br.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        header_depthimage = data.header
        self.depthimage = cv_depth
        projection(self.x,self.y,self.self.depthimage)
        
    def projection(self):
        if self.coord3d.x is not None and self.depthimage is not None:
            cx = self.image.x
            cy = self.image.y
            img = self.depthimage
            #TODO: get drone pose

            cx = cx.reshape(-1)
            cy = cy.reshape(-1)
            mask = cx<480
            mask=mask&(cx>=0)
            mask=mask&(cy<640)
            mask=mask&(cy>=0)
            cx = cx[mask]
            cy = cy[mask]

            cx=np.int32(cx)
            cy=np.int32(cy)

            X = np.vstack((cx,cy,np.ones(len(cx))))
            #TODO: get K from camera_info
            # k_int = np.array([[]])
            k_int = np.array([[465.60148469763925, 0.0, 320.5],
                                [0.0, 465.60148469763925, 240.5],
                                [0.0, 0.0, 1.0]])
            X=np.matmul(np.linalg.inv(k_int),X)
            
            unit_vec=X/np.linalg.norm(X,axis=0)
            dep=img[cx,cy]
            new_vec=unit_vec*dep
            quaternion = (drone_pose.pose.orientation.x,drone_pose.pose.orientation.y,drone_pose.pose.orientation.z,drone_pose.pose.orientation.w)
            mat = tf.transformations.quaternion_matrix(quaternion)
            transform_mat=mat
            transform_mat[:3,3]=[drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
            new_vec_p = np.vstack((new_vec, np.ones(new_vec.shape[1])))
            transform_mat_c_d = np.array([[0., -1, 0., 0.],
                                            [-1., 0., 0., 0.],
                                            [0., 0., -1., 0.],
                                            [0., 0., 0., 1.]])
            coord = np.matmul(transform_mat_c_d, new_vec_p)
            coord=np.matmul(transform_mat, coord)
            print(coord.shape)
            # return coord


def main(args=None):
    rclpy.init(args=args)

    node_ = TransformationNode()

    rclpy.spin(node_)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()