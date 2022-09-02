#!/usr/bin/env pyton3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from px4_msgs.msg import Timesync
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
#check where the positioin of red ball is being published

ball_x = 1.0   # random values for ball_x, ball_y, ball_z
ball_y = 1.0
ball_z = 0.27
r_horz = 0.6
r_vert = 0.8
class   Trajectory_drone_Node(Node):

    def __init__(self):
        super().__init__('drone_publisher')
        self.timestamp = None
        self.x_drone = 0
        self.y_drone = 0
        self.z_drone = 0
        self.drone_pose_publish = self.create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10) #check for topic which will publish it to the drone
        self.subs_red_ball_pos = self.create_subscription(Pose, '/Car_pose',self.red_ball_callback,100) #update this after checking where the position of ball is being published
        self.time_subscription = self.create_subscription(Timesync,'/Timesync_PubSubTopic',self.timestamp_callback, 1)
        self.publisher_offboard = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.timer_offboard = self.create_timer(0.01, self.offboard_publisher)
        # self.subs_red_ball_pos #prevents unused variable warning
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

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

    def timestamp_callback(self,msg):
        self.timestamp = msg.timestamp #CHECK THIS IF ANY ERROR OCCURS
    def red_ball_callback(self, msg):
        self.ball_x = msg.position.x
        self.ball_y = msg.position.y
        self.ball_z = msg.position.z
        self.get_logger().info('Received red ball position \n'+str(msg))
    
    def opt_traj_publisher(self):
        drone_pos = TrajectorySetpoint()
        drone_pos.x = self.ball_x
        drone_pos.y = self.ball_y + r_horz
        drone_pos.z = self.ball_z + r_vert

        self.drone_pose_publish.publish(drone_pos)


def main(args=None):
    rclpy.init(args=args)

    traj_drone = Trajectory_drone_Node()
    rclpy.spin(traj_drone)
    traj_drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()