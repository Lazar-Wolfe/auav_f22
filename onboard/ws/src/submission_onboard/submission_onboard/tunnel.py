#!/usr/bin/env python3
import cv2 
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge,CvBridgeError
from px4_msgs.msg import VehicleLocalPosition,VehicleAttitude
from geometry_msgs.msg import Pose



class TunnelNode(Node):
    def __init__(self):
        super().__init__('tunnel_node')
        self.rgbimage = self.depthimage = None
        self.subscription = self.create_subscription(VehicleAttitude, '/VehicleAttitude_PubSubTopic', self.drone_orientation_callback, 100)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/VehicleLocalPosition_PubSubTopic',self.drone_odom_callback,100)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/image',self.rgbimage_callback,100)
        self.subscription = self.create_subscription(Image,'/rgbd_camera/depth_image',self.depthimage_callback,100)
        self.isTunnel = False
        self.br = CvBridge()
        self.x_drone = self.y_drone = self.z_drone = None
        self.orientationx = self.orientationy = self.orientationz = self.orientationw = None


    def rgbimage_callback(self,data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.rgbimage = cv_image
        self.tunnel_detecion()

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
    
    def depthimage_callback(self,data):
        try:
            cv_depth = self.br.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.depthimage = cv_depth
    
    def tunnel_detecion(self):
        if self.rgbimage is not None and self.depthimage is not None:
            frame = self.rgbimage
            # print(frame.shape)
            # frame = frame[240:, 160:480]
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            ret,thresh = cv2.threshold(gray,9,255,cv2.THRESH_BINARY_INV)
            contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

            # print(len(contours))
            # min_area=40
            # max_area=400*600

            # can also try area ratio of contour/bounding box would be less than 1
            for cnt in contours:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                area = cv2.contourArea(cnt)
                approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
                area = cv2.contourArea(cnt)
                area_rect = cv2.contourArea(box)
                if area_rect==0:
                    continue
                area_ratio = area/area_rect
                print(f"ratio{area/area_rect}")
                # if area>min_area and area<max_area:
                # cv2.drawContours(frame,[approx],0,(0,255,0),2)
                cv2.drawContours(frame,[box],0,(0,255,255),2)
                cv2.drawContours(frame,cnt,-1,(0,0,255),2)
                m = cv2.moments(cnt)
                cx = m['m10'] / m['m00']
                cy = m['m01'] / m['m00']
                inside = cx>150 and cx<550 and cy>160
                if not inside:
                    # print("Outsider")
                    continue
                if area_ratio<0.9:
                    k = True
                    cv2.circle(frame, (int(cx), int(cy)), 5, (255, 0, 0), -1)
                else:
                    k = False
                if k:
                    print("Tunnel Detected")
                    self.isTunnel = True
                    print(f"Distance to collision {self.depthimage[int(cy)][int(cx)]}")
                # print(m)
            cv2.imshow('mask', frame)
            cv2.waitKey(1)
            self.rgbimage = None

def main(args=None):
    rclpy.init(args=args)

    tunnel_node = TunnelNode()

    rclpy.spin(tunnel_node)
    tunnel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

