import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

INDEX = 100 

class Sample_traj_car(Node):

    def __init__(self):
        super().__init__('car_node')
        self.car_pose_subscriber = self.create_subscription(Odometry,'/odom',self.circular_traj,10)
        self.car_pose_publisher = self.create_publisher(Pose,'/Car_pose',10)
        # self.timer = self.create_timer(1,self.circular_traj)
        self.i = 0

    def circular_traj(self,msg):
        car_pose = Pose()
        car_pose.position.x = -msg.pose.pose.position.y - 1
        car_pose.position.y = msg.pose.pose.position.x + 1.2
        car_pose.position.z = msg.pose.pose.position.z
        # self.i = self.i%100 + 1    #increasing the value of i by 1 and then taking the remainder by 100
        self.car_pose_publisher.publish(car_pose)




def main(args=None):
    rclpy.init(args=args)
    car_node = Sample_traj_car()
    rclpy.spin(car_node)
    car_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()