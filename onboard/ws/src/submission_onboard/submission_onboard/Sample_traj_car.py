import rclpy
import math
from rclpy.node import Node
from geometry_msgs import Pose

INDEX = 100

class Sample_traj_car(Node):

    def __init__(self):
        super().__init__('car_node')
        self.car_pose_publisher = self.creat_publisher(Pose,'/Car_pose',self.car_odom_callback,10)
        self.timer = self.create_timer(0.01,self.circular_traj)
        self.i = 0

    def circular_traj(self):
        car_pose = Pose()
        car_pose.position.x = 1.0 + 3*math.cos(2*math.pi*self.i*0.01)
        car_pose.position.y = 1.0 + 3*math.sin(2*math.pi*self.i*0.01)
        car_pose.position.z = 0.2
        self.i = self.i%100 + 1
        self.car_pose_publisher.publish(car_pose)



def main(args=None):
    rclpy.init(args=args)
    car_node = Sample_traj_car()
    rclpy.spin(car_node)
    car_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()