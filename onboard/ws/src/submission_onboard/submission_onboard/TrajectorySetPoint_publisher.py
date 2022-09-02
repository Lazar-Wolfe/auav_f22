#!/usr/bin/env pyton3
import rclpy
import math
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Pose


class SetPoint_Traj_Node(Node):

    def __init__(self):
        super().__init__('drone_node')
        self.timestamp = None
        self.x_drone = 0
        self.y_drone = 0
        self.z_drone = 0
        self.X_car = 1.0
        self.Y_car = 1.0
        self.Z_car = 5.0
        self.counter = 0
        self.subscription_time = self.create_subscription(Timesync,'/Timesync_PubSubTopic',self.timestamp_callback, 10)
        self.publisher_offboard = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/VehicleCommand_PubSubTopic', 10)
        self.publisher_setpoint = self.create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10)
        self.timer_traj = self.create_timer(0.01, self.timer_callback)
        self.subscription_pos = self.create_subscription(VehicleLocalPosition,'/VehicleLocalPosition_PubSubTopic',self.drone_odom_callback,100)
        self.subscriber_car = self.create_subscription(Pose,'/Car_pose',self.car_pose,100)
        
    def traj_publisher(self):
        # delta_x = self.X_car-self.x_drone 
        # delta_y = self.Y_car-self.y_drone
        # dist = math.sqrt(delta_x**2+delta_y**2) 
        # x_des = self.x_drone+0.6*delta_x/dist 
        # y_des = self.y_drone+0.6*delta_y/dist
        # z_des = self.Z_car + 0.8
        x_des=self.X_car
        y_des=self.Y_car - (1.0*2-0.75*2)**0.5
        z_des=-1*(self.Z_car+0.75)
        if self.timestamp is not None:
            try_point = TrajectorySetpoint()
            try_point.timestamp = self.timestamp
            try_point.x = x_des
            try_point.y = y_des
            try_point.z = -z_des

            # self.get_logger().info('Published trajectory \n'+str(try_point))

            # self.get_logger().info('Publishing point')
            self.publisher_setpoint.publish(try_point)
            # self.get_logger().info('Published point')

    def timer_callback(self):
        self.offboard_publisher()
        self.traj_publisher()

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
            if self.counter < 10:
                self.counter += 1
            elif self.counter == 10:
                self.get_logger().info("counter gone to 10")
                self.publish_vehicle_command(VehicleCommand().VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                # self.publish_vehicle_command(VehicleCommand().VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.counter += 1
        # self.get_logger().info('Published offboard \n'+str(offboard_msg))

    def publish_vehicle_command(self, command, param1 = 0.0, param2 = 0.0):
        vehicle_cmd_msg = VehicleCommand()
        vehicle_cmd_msg.timestamp = self.timestamp
        vehicle_cmd_msg.param1 = param1
        vehicle_cmd_msg.param2 = param2
        vehicle_cmd_msg.command = command
        vehicle_cmd_msg.target_system = 1
        vehicle_cmd_msg.target_component = 1
        vehicle_cmd_msg.source_system = 1
        vehicle_cmd_msg.source_component = 1
        vehicle_cmd_msg.from_external = True

        self.publisher_vehicle_command.publish(vehicle_cmd_msg)

    def drone_odom_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.x_drone = msg.x
            self.y_drone = msg.y
            self.z_drone = -msg.z
            # self.get_logger().info('got current coords'+"\n"+str(msg))


    def timestamp_callback(self, msg):
        self.timestamp = msg.timestamp
        # self.get_logger().info('Got time stamp'+"\n"+str(msg))
    
    def car_pose(self, msg):
        self.X_car = msg.position.x
        self.Y_car = msg.position.y
        self.Z_car = msg.position.z
        # self.get_logger().info('got car coords'+"\n"+str(msg))

def main(args=None):
    rclpy.init(args=args)

    setpoint_traj = SetPoint_Traj_Node()

    rclpy.spin(setpoint_traj)
    setpoint_traj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()