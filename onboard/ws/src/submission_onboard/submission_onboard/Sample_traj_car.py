import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose

INDEX = 100 

class Sample_traj_car(Node):

    def __init__(self):
        super().__init__('car_node')
        self.car_pose_publisher = self.create_publisher(Pose,'/Car_pose',10)
        self.timer = self.create_timer(0.01,self.circular_traj)
        self.i = 0

    def circular_traj(self):
        car_pose = Pose()
        car_pose.position.x = 0.0 + 3*math.cos(2*math.pi*self.i*0.01)
        car_pose.position.y = 2.0 + 3*math.sin(2*math.pi*self.i*0.01)
        car_pose.position.z = 0.095 #radius of tire is 0.055 and 0.08 is the width of the car, so the height should be around 0.19 or less
        self.i = self.i%100 + 1    #increasing the value of i by 1 and then taking the remainder by 100
        self.car_pose_publisher.publish(car_pose) #publishing the car pose



def main(args=None):
    rclpy.init(args=args)
    car_node = Sample_traj_car()
    rclpy.spin(car_node)
    car_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()