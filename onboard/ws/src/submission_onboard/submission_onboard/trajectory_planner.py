#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math 
import numpy as np
from nav_msgs.msg import Odometry #this will help to get to the data published on the /odom topic
from geometry_msgs.msg import Pose #this will help to get the car pose
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from rclpy.node import Node
import tf_transformations as tf

class trajectory_Publisher(Node):

    def __init__(self):
        super().__init__('trajectory_Publisher')
        self.r=1.0
        self.h=0.75
        self.timestamp = None
        self.ball_y=0.0
        self.ball_z=0.0
        self.ball_yaw=0.0
        self.ball_roll=0.0
        self.ball_pitch=0.0
        self.drone_x=None
        self.drone_y=None
        self.drone_z=None
        self.drone_yaw=None
        self.counter = 0
        self.subscription_time = self.create_subscription(Timesync,'/Timesync_PubSubTopic',self.timestamp_callback, 10)
        self.publisher_offboard = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/VehicleCommand_PubSubTopic', 10)
        self.timer_offboard = self.create_timer(0.01, self.offboard_publisher)
        
        self.trajectory_points_publish = self.create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10)
        self.car_pose_subscriber = self.create_subscription(Pose,'/Car_pose',self.pose_callback,10)#this will subscribe the data from the topic /Car_pose to get the current position of the car
        self.timer = self.create_timer(0.1, self.traj_publisher_callback)
        self.i = 0
        
    def trajectory_generator(self):#This function will just derive the new trajectory coordinate of the drone based on the current position of the car(without considering any obstacle, buildings or tunnels nearby)
        self.drone_x=self.ball_x
        self.drone_y=self.ball_y - (self.r*2-self.h*2)**0.5
        self.drone_z=self.ball_z+self.h
        self.drone_yaw=self.ball_yaw

    def timestamp_callback(self, msg):
        self.timestamp = msg.timestamp
        # self.get_logger().info('Got time stamp'+"\n"+str(msg))
            
        
    def pose_callback(self, msg):#This function will be called whenever any new msg arrive on the /Car_pose topic and the save the topic data to our function's vairable for their use in trajectory generation.
        self.ball_x=msg.position.x
        self.ball_y=msg.position.y
        self.ball_z=msg.position.z
        a=msg.orientation.x
        b=msg.orientation.y
        c=msg.orientation.z
        d=msg.orientation.w

        euler_angles = tf.euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        self.ball_roll=euler_angles[0]
        self.ball_pitch=euler_angles[1]
        self.ball_yaw=euler_angles[2]
        self.trajectory_generator()
        #self.get_logger().info('' % msg.data)

    def traj_publisher_callback(self):
        if self.drone_x is not None and self.drone_y is not None and self.drone_yaw is not None and self.drone_z is not None:
            msg = TrajectorySetpoint()
            msg.x=self.drone_x
            print(f"YYYY{self.drone_y}")
            msg.y=self.drone_y
            msg.z=self.drone_z
            msg.yaw=self.drone_yaw
            self.trajectory_points_publish.publish(msg)
            self.get_logger().info('Publishing: drone_x "%f"' % msg.x)
            self.get_logger().info('Publishing: drone_y "%f"' % msg.y)
            self.get_logger().info('Publishing: drone_z "%f"' % msg.z)
            self.get_logger().info('Publishing: drone_Yaw "%f"' % msg.yaw)
            #self.i += 1
            self.drone_x = self.drone_y = self.drone_yaw = self.drone_z = None
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
                self.counter += 1
        # self.get_logger().info('Published offboard \n'+str(offboard_msg))
    def timer_callback(self):
        self.offboard_publisher()
        self.traj_publisher()

    def publish_vehicle_command(self, command, param1, param2):
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


def main(args=None):
    rclpy.init(args=args)
    
    trajectory = trajectory_Publisher()
    rclpy.spin(trajectory)
    trajectory.destroy_node()

    
    rclpy.shutdown()


if __name__ == '__main__':
    main()