#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import cv2, numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge,CvBridgeError
import tf
import geometry_msgs
import tf.transformations
from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_matrix


def projection(cx,cy,img,K=None):
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
    global drone_pose
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
    return coord


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
    return mask_3,int(y[1]),int(y[0])
 


class DetectorNode(Node):

    def __init__(self):
        super().__init__('drone_node')

        self.publisher_ = self.create_publisher(Image, 'detector_topic', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publisher_function)

        self.br = CvBridge()
        self.rgbimage = None
        self.depthimage = None
        self.ans = None
        self.x = None
        self.y = None
        self.subscription = self.create_subscription(Image,'/rgbd_camera/image',self.rgbimage_callback,10)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/depth_image',self.depthimage_callback,10)
        # self.subscription = self.create_subscription(P,'/odom',self.odom_callback,10)

    def rgbimage_callback(self,data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        header_rgbimage = data.header
        self.rgbimage = cv_image
        self.ans,self.x,self.y = red_detector(self.rgbimage)


    def depthimage_callback(self,data):
        try:
            cv_depth = self.br.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        header_depthimage = data.header
        self.depthimage = cv_depth
        self.car_pose = projection(self.x,self.y,self.self.depthimage)

        

    def publisher_function(self):
        if self.ans is not None:
            # img = cv2.resize(self.rgbimage,(300,300))
            if self.depthimage is not None:
                img = self.ans
                image_message = Image()
                image_message = self.br.cv2_to_imgmsg(img, encoding="bgr8")
                self.publisher_.publish(image_message)
                self.get_logger().info('Publishing new image')
                self.car_pose=projection()
                self.ans = None


def main(args=None):
    rclpy.init(args=args)

    drone_node = DetectorNode()

    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()