import rclpy
from rclpy.node import Node
import math 
import numpy as np
#import matplotlib
#import pandas
from nav_msgs.msg import Odometry #this will help to get to the data published on the /odom topic
from geometry_msgs.msg import Pose #this will help to get the car pose
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from rclpy.node import Node

class trajectory_Publisher(Node):

    def __init__(self):
        super().__init__('trajectory_Publisher')
        self.r=1.0
        self.h=1.5
        self.drone_x=0.0
        
        
        self.ball_y=0.0
        self.ball_z=0.0
        self.ball_yaw=0.0
        self.ball_roll=0.0
        self.ball_pitch=0.0
        self.drone_x=0.0
        self.drone_y=0.0
        self.drone_z=0.0
        self.drone_yaw=0.0
        self.trajectory_points_publish = self.create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubsubTopic', 10)
        
        self.car_pose_subscriber = self.create_subscription(
            Pose,
            '/Car_pose',
            self.pose_callback,
            10)#this will subscribe the data from the topic /Car_pose to get the current position of the car
        self.timer = self.create_timer(0.01, self.traj_publisher_callback)
        self.i = 0
        
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
    def traj_setpoint_final(self,x_drone,y_drone,z_drone,yaw_drone):#this funtion will check for the distance between the car and the nearby buildings and then implement the required algorithm to find the new trajectory points for the quadrotor
        self.drone_x=x_drone
        self.drone_y=y_drone
        self.drone_z=y_drone
        self.drone_yaw=yaw_drone
    def trajectory_generator(self,ball_x,ball_y,ball_z,ball_roll,ball_pitch,ball_yaw):#This function will just derive the new trajectory coordinate of the drone based on the current position of the car(without considering any obstacle, buildings or tunnels nearby)
        x_drone=ball_x
        y_drone=ball_y - (self.r*2-self.h*2)**0.5
        z_drone=ball_z+self.h
        yaw_drone=ball_yaw
        self.traj_setpoint_final(x_drone,y_drone,z_drone,yaw_drone,self)

  
            
        
    def pose_callback(self, msg):#This function will be called whenever any new msg arrive on the /Car_pose topic and the save the topic data to our function's vairable for their use in trajectory generation.
        self.ball_x=msg.position.x
        self.ball_y=msg.position.y
        self.ball_z=msg.position.z
        euler_angles=self.euler_from_quaternion(float(msg.orientation.x),msg.orientation.y,msg.orientation.z,msg.orientation.w)
        self.ball_roll=euler_angles[0]
        self.ball_pitch=euler_angles[1]
        self.ball_yaw=euler_angles[2]
        self.trajectory_generator(self.ball_x,self.ball_y,self.ball_z,self.ball_roll,self.ball_pitch,self.ball_yaw,self)
        #self.get_logger().info('' % msg.data)

    def traj_publisher_callback(self):
        msg = TrajectorySetpoint()
        msg.x=self.drone_x
        msg.y=self.drone_y
        msg.z=self.drone_z
        msg.yaw=self.drone_yaw
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: drone_x "%f"' % msg.x)
        self.get_logger().info('Publishing: drone_y "%f"' % msg.y)
        self.get_logger().info('Publishing: drone_z "%f"' % msg.z)
        self.get_logger().info('Publishing: drone_Yaw "%f"' % msg.yaw)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)
    
    trajectory = trajectory_Publisher()
    rclpy.spin(trajectory)
    trajectory.destroy_node()

    
    rclpy.shutdown()


if __name__ == '__main__':
    main()