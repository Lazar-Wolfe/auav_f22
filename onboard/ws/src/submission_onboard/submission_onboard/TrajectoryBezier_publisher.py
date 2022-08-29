#!/usr/bin/env pyton3
import rclpy
import math
from rclpy.node import Node
from px4_msgs.msg import TrajectoryBezier
from px4_msgs.msg import VehicleLocalPosition

X_car = 1.0
Y_car = 1.0
Z_car = 0.1

class DetectorNode(Node):

    def __init__(self):
        super().__init__('drone_node')
        self.publisher_bezier = self.create_publisher(TrajectoryBezier, '/TrajectoryBezier_PubSubTopic', 10)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/VehicleLocalPosition_PubSubTopic',self.drone_odom_callback,10)
        self.x_drone = 0
        self.y_drone = 0
        self.z_drone = 0

    def traj_publisher(self):
        delta_x = X_car-self.x_drone 
        delta_y = Y_car-self.y_drone
        dist = math.sqrt(delta_x**2+delta_y**2) 
        # x_des = self.x_drone*delta_x/dist 
        # y_des = self.y_drone*delta_y/dist
        # z_des = Z_car + 0.8

        x_des = 5.0
        y_des = 5.0
        z_des = 10.0

        try_point = TrajectoryBezier()
        timestamp = self.get_clock().now().to_msg()
        try_point.timestamp = timestamp.sec
        try_point.position = [x_des, y_des, z_des]
        try_point.delta = 1.0
        self.get_logger().info('Publishing point')
        # self.get_logger().info(try_point)
        self.publisher_bezier.publish(try_point)

    def drone_odom_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.x_drone = -msg.x
            self.y_drone = -msg.y
            self.z_drone = -msg.z
            self.traj_publisher()

def main(args=None):
    rclpy.init(args=args)

    drone_node = DetectorNode()

    rclpy.spin_once(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()