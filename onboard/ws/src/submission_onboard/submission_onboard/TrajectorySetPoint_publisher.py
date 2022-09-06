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

cntr = 0
previousx = [0.,0.,0.]
previousy = [0.,0.,0.]
class SetPoint_Traj_Node(Node):

    def __init__(self):
        super().__init__('drone_node')
        self.timestamp = None
        self.kp=0.5
        self.kd=0.5
        self.t_sampling=1

        self.x_drone =None
        self.y_drone =None
        self.z_drone =None
        self.drone_yaw=None

        self.X_car = None
        self.Y_car = None
        self.Z_car = 0.
        self.yaw_car=0.
        self.ang_drn_rvr = None
        self.counter =0

        self.h = 0.4
        self.r = math.sqrt(1-self.h**2)
        self.R = 1.0

        self.x_err_fct = 1

        self.y_err_fct = 1

        self.car_x_new=None
        self.car_y_new=None
        self.car_z_new=None


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
        print("Just inside")
        if self.X_car is not None and self.Y_car is not None :
            print("inside trajpublisher")
            # delta_x = self.X_car-self.x_drone
            # delta_y = self.Y_car-self.y_drone
            # dist = math.sqrt(delta_x**2+delta_y**2)
            # a = dist-self.r
            # x_des = (self.x_drone*self.r+self.X_car*a)/dist
            # y_des = (self.y_drone*self.r+self.Y_car*a)/dist
            # z_des = self.Z_car + self.h
            # yaw_des = math.atan2(delta_y,delta_x)
            
            #PRANJAL
            y_des=self.Y_car + self.y_err_fct*(((self.R)**2-(self.h)**2)**0.5)*math.cos(self.yaw_car)
            x_des=self.X_car + self.x_err_fct*(((self.R)**2-(self.h)**2)**0.5)*math.sin(self.yaw_car)
            z_des=(self.Z_car + self.h)
            self.car_x_new=self.X_car - self.x_drone
            self.car_y_new=self.Y_car - self.y_drone
            self.ang_drn_rvr =math.atan2(self.car_y_new,self.car_x_new)
            self.yaw_des =self.ang_drn_rvr # this works in all the cases of quadrants, yaw_des is the final required yaw of drone

            self.drone_yaw = self.yaw_des

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
                try_point.yaw = self.yaw_des
            

                # try_point.vx = x_dot_des
                # try_point.vy = y_dot_des
                # try_point.vz = z_dot_des

                # self.get_logger().info('Published trajectory \n'+str(try_point))
                # self.get_logger().info('Publishing point')
                self.publisher_setpoint.publish(try_point)
                # self.get_logger().info('Published point')
            self.timestamp = None
            self.X_car =self.Y_car = self.Z_car = None

            

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
            self.drone_yaw = msg.heading
            # self.get_logger().info('got current coords'+"\n"+str(msg))


    def timestamp_callback(self, msg):
        self.timestamp = msg.timestamp
        # self.get_logger().info('Got time stamp'+"\n"+str(msg))
    
    def car_pose(self, msg):
        global cntr
        print("Gopt car pose")
        self.X_car = msg.position.y
        self.Y_car = msg.position.x
        self.Z_car = - msg.position.z
        if len(previousx) > 3:
            errx = abs(previousx[-1]-self.X_car)
            erry = abs(previousy[-1]-self.Y_car)
            if errx < 1.5 and erry < 1.5:
                previousx.append(self.X_car)
                previousy.append(self.Y_car)
        # self.get_logger().info('got car coords'+"\n"+str(msg))
            else:
                cntr =+ 1
            if cntr > 5: #here we can change the numbers of the differences to only 2 differnces
                # avg_diff_errx = ((previousx[-1]-previousx[-2]) + (previousx[-2] - previousx[-3]) + (previousx[-3] - previousx[-4]) + (previousx[-4] - previousx[-5]))/4
                # avg_diff_erry = ((previousy[-1]-previousy[-2]) + (previousy[-2] - previousy[-3]) + (previousy[-3] - previousy[-4]) + (previousy[-4] - previousy[-5]))/4
                # self.X_car = previousx[-1] + avg_diff_errx
                # self.Y_car = previousy[-1] + avg_diff_erry
                self.X_car = msg.pose.position.y
                self.Y_car = msg.pose.position.x
                self.Z_car = -msg.pose.position.z # no error in z till now
                previousx.append(self.X_car)
                previousy.append(self.Y_car)
            if len(previousx) > 10:
                previousx.pop(0)
                previousy.pop(0)
        else:
            previousx.append(self.X_car)
            previousy.append(self.Y_car)
            self.X_car = msg.position.y
            self.Y_car = msg.position.x
            self.Z_car = - msg.position.z
        

def main(args=None):
    rclpy.init(args=args)

    setpoint_traj = SetPoint_Traj_Node()

    rclpy.spin(setpoint_traj)
    setpoint_traj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()