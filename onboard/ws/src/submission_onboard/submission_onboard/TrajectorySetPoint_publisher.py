#!/usr/bin/env pyton3
import rclpy
import math
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import OffboardControlMode

X_car = 1.0
Y_car = 1.0
Z_car = 0.1

class SetPoint_Traj_Node(Node):

    def __init__(self):
        super().__init__('drone_node')
        self.timestamp = None
        self.x_drone = 0
        self.y_drone = 0
        self.z_drone = 0
        self.subscription_pos = self.create_subscription(VehicleLocalPosition,'/VehicleLocalPosition_PubSubTopic',self.drone_odom_callback,10)
        self.subscription_time = self.create_subscription(Timesync,'/Timesync_PubSubTopic',self.timestamp_callback, 1)
        self.publisher_setpoint = self.create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10)
        self.publisher_offboard = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.timer_offboard = self.create_timer(0.01, self.offboard_publisher)
        self.timer_traj = self.create_timer(0.1, self.traj_publisher)
        
    def traj_publisher(self):
        delta_x = X_car-self.x_drone 
        delta_y = Y_car-self.y_drone
        dist = math.sqrt(delta_x**2+delta_y**2) 
        x_des = self.x_drone+0.35*delta_x/dist 
        y_des = self.y_drone+0.35*delta_y/dist
        z_des = Z_car + 0.8

        try_point = TrajectorySetpoint()
        try_point.timestamp = self.timestamp
        try_point.x = x_des
        try_point.y = y_des
        try_point.z = -z_des

        # self.get_logger().info('Published trajectory \n'+str(try_point))

        # self.get_logger().info('Publishing point')
        self.publisher_setpoint.publish(try_point)
        # self.get_logger().info('Published point')

        
    def offboard_publisher(self):
        if self.timestamp is not None:
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = self.timestamp
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            self.publisher_offboard.publish(offboard_msg)
        # self.get_logger().info('Published offboard \n'+str(offboard_msg))

    def drone_odom_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.x_drone = msg.x
            self.y_drone = msg.y
            self.z_drone = -msg.z
            # self.get_logger().info('got current coords'+"\n"+str(msg))


    def timestamp_callback(self, msg):
        self.timestamp = msg.timestamp
        # self.get_logger().info('Got time stamp'+"\n"+str(msg))

def main(args=None):
    rclpy.init(args=args)

    setpoint_traj = SetPoint_Traj_Node()

    rclpy.spin(setpoint_traj)
    setpoint_traj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()