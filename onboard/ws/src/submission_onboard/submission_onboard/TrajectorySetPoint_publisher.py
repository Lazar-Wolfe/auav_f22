#!/usr/bin/env pyton3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
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
        self.kp=0.5
        self.kd=0.5
        self.t_sampling=1

        self.x_drone = 0
        self.y_drone = 0
        self.z_drone = 0

        self.X_car = 0.0
        self.Y_car = 0.0
        self.Z_car = 0.0
        self.yaw_car=0.0
        self.counter = 0

        self.h = 0.8
        self.r = math.sqrt(1-self.h**2)

        # self.drone_curr_x=-3.0
        # self.drone_curr_y=0.0
        # self.drone_curr_z=0.06

        # self.drone_curr_vel_x=0.0
        # self.drone_curr_vel_y=0.0
        # self.drone_curr_vel_z=0.0

        self.subscription_time = self.create_subscription(Timesync,'/Timesync_PubSubTopic',self.timestamp_callback, 10)
        self.publisher_offboard = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/VehicleCommand_PubSubTopic', 10)
        self.publisher_setpoint = self.create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10)
        self.timer_traj = self.create_timer(1, self.timer_callback)
        self.subscription_pos = self.create_subscription(VehicleLocalPosition,'/VehicleLocalPosition_PubSubTopic',self.drone_odom_callback,100)
        #self.subscription_drone_pose_twist=self.create_subscription(Odometry,'/odom',self.velocity_controller,100)#this will subscribe the current drone position and velocity 
        self.subscriber_car = self.create_subscription(Pose,'/Car_pose',self.car_pose,10)

    # def velocity_controller(self,msg):
    #     self.drone_curr_x=msg.pose.pose.position.x
    #     self.drone_curr_y=msg.pose.pose.position.y
    #     self.drone_curr_z=msg.pose.pose.position.z

    #     self.drone_curr_vel_x=msg.twist.twist.linear.x
    #     self.drone_curr_vel_y=msg.twist.twist.linear.y
    #     self.drone_curr_vel_z=msg.twist.twist.linear.z
        
    def traj_publisher(self):

        # delta_x = self.X_car-self.x_drone
        # delta_y = self.Y_car-self.y_drone
        # dist = math.sqrt(delta_x**2+delta_y**2)
        # a = dist-self.r
        # x_des = (self.x_drone*self.r+self.X_car*a)/dist
        # y_des = (self.y_drone*self.r+self.Y_car*a)/dist
        # z_des = self.Z_car + self.h
        # yaw_des = math.atan2(delta_y,delta_x)
        
        #PRANJAL
        y_des=self.Y_car + ((1.0*2-0.65*2)**0.5)*math.cos(self.yaw_car)
        x_des=self.X_car+((1.0*2-0.65*2)**0.5)*math.sin(self.yaw_car)
        z_des=(self.Z_car + 0.65)
        yaw_des = self.yaw_car

        # err_x=x_des-self.drone_curr_x
        # err_y=y_des-self.drone_curr_y
        # err_z=z_des-self.drone_curr_z

        # if err_x<0.5 and err_y<0.5 and err_z<0.5:
        #     x_dot_des=0
        #     y_dot_des=0
        #     z_dot_des=0   
        # else:
        #     x_dot_des=self.drone_curr_vel_x/self.t_sampling + self.kp*err_x
        #     y_dot_des=self.drone_curr_vel_y/self.t_sampling + self.kp*err_y
        #     z_dot_des=self.drone_curr_vel_z/self.t_sampling + self.kp*err_z


        # print(f"Going to {x_des} {y_des} {z_des}")
        if self.timestamp is not None:
            try_point = TrajectorySetpoint()
            try_point.timestamp = self.timestamp
            try_point.x =  x_des
            try_point.y = y_des
            try_point.z = -z_des
            try_point.yaw = yaw_des

            # try_point.vx = x_dot_des
            # try_point.vy = y_dot_des
            # try_point.vz = z_dot_des


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
                self.publish_vehicle_command(VehicleCommand().VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
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
            self.get_logger().info('got current coords'+"\n"+str(msg))


    def timestamp_callback(self, msg):
        self.timestamp = msg.timestamp
        # self.get_logger().info('Got time stamp'+"\n"+str(msg))
    
    def car_pose(self, msg):
        self.X_car = msg.position.y
        self.Y_car = msg.position.x
        self.Z_car = - msg.position.z
        # self.get_logger().info('got car coords'+"\n"+str(msg))

def main(args=None):
    rclpy.init(args=args)

    setpoint_traj = SetPoint_Traj_Node()

    rclpy.spin(setpoint_traj)
    setpoint_traj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()