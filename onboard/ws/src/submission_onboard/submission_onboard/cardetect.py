#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2, numpy as np
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

# def car_detector(frame,depth):
fss = 0
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
            # self.ans,self.x,self.y = self.car_detector()
            self.car_detector()
        except:
            self.x,self.y=-999,-999
            print("Publishing -999")

    def depthimage_callback(self,data):
        try:
            cv_depth = self.br.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.depthimage = cv_depth

    def car_detector(self):
        if self.rgbimage is not None:
            print("Detecting car")
            img = self.rgbimage
            # cv2.imshow("img,",img)
            # cv2.waitKey(1)
            print(img.shape)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            ret,thresh = cv2.threshold(gray,9,255,cv2.THRESH_BINARY_INV)
            # cv2.imshow("thresh",thresh)
            # cv2.waitKey(1)
            contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

            # print(len(contours))
            min_area=500
            max_area=5000
            low_car_white = np.array([0,37,1])
            high_car_white = np.array([79,135,154])
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
                    # cv2.drawContours(img,[approx],0,(0,255,0),2)
                    M = cv2.moments(approx)
                    if M['m00'] != 0:
                        # print("HJELKSJEDFKLJD")
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        # cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)
                        # cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)
                        # print("centre of car",cx,cy)
                        roi = img[cy-75:cy+75,cx-75:cx+75]
                        # cv2.imshow("roi",roi)
                        # cv2.waitKey(1)
                        # cv2.imwrite("carroihe.jpg",roi)
                        # fss+=1
                        hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
                        mask = cv2.inRange(hsv,low_car_white,high_car_white)
                        # print("mask shaoe" ,hsv.shape)
                        # print("IDHAR")
                        mask_3 = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
                        _,th = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
                        white = np.argwhere(th == 255)
                        white = np.transpose(white)
                        if white.size != 0:
                            
                            y = np.mean(white,axis=1)
                            cv2.circle(mask_3,(int(y[1]),int(y[0])),2,(0,0,255),3)
                            # y[0] is down
                            # y[1] is right
                            y[0]=y[0]-75+cy
                            y[1]=y[1]-75+cx
                            self.x = y[1]
                            self.y = y[0]
                            print(y)
                            # print(mask_3.shape)
                            cv2.imshow("MASK#3",mask_3)
                            # cv2.imshow("ACTUALIMGAEW",img)
                            cv2.waitKey(1)
                        # key = cv2.waitKey(30)
                        # if key == ord('q') or key == 27:
                        #     break
            # cv2.drawContours(img, contours, -1,(0,0,255),3)
            # cv2.imshow("cAR",img)
            # cv2.waitKey(1)
            # cv2.imshow("cAR",thresh)
            # cv2.waitKey(100)
            # return cx,cy,0



def main(args=None):
    rclpy.init(args=args)
    car_detector_node = DetectorNode()
    rclpy.spin(car_detector_node)
    car_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()