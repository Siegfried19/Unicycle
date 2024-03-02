import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt

import math
class Subscribe(Node):
    def __init__(self):
        super().__init__('Subscribe')
        self.x = 0
        self.y = 0
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
    	
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        print(self.x,self.y)

class CircularTrajectoryController(Node):
    def __init__(self,goal):
        super().__init__('circular_trajectory_controller')
        self.goal = goal
        self.i = 0
        self.x_goal = goal [0][0]
        self.y_goal = goal [0][1]
        self.x_current = 0
        self.y_current = 0
        self.actual_x = []  # List to store actual x positions
        self.actual_y = []  # List to store actual y positions
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.target_radius = 1.0  # meters
        self.center_x = 0  # x coordinate of the circle's center
        self.center_y = 0  # y coordinate of the circle's center
        self.Kp = 3  # proportional gain
        self.Kd = 0.3  # derivative gain
        self.Kp_angle = 5 # proportional angular gain
        self.Kd_angle = 0.5
        self.prev_distance = 0.0
        self.prev_angle = 0.0
        self.arrival_threshold = 0.001
        self.time_now = time.time()
        self.time_last = time.time()
        self.frequency = 0.01 # 100hz, time interval 0.01s
        
    def odom_callback(self, msg):
        # Current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
	
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
	
        # Calculate the nearest point on the circle to the robot
        #angle_to_center = math.atan2(y - self.center_y, x - self.center_x)
        #target_x = self.center_x + self.target_radius * math.cos(angle_to_center)
        #target_y = self.center_y + self.target_radius * math.sin(angle_to_center)
       
        
        # Calculate the error as the distance to the target point on the circle
        distance = math.sqrt((self.x_goal - x) ** 2 + (self.y_goal - y) ** 2)
        

	# Calculate the error as the angle to the target point on the circle
        angle_to_goal = math.atan2(self.y_goal - y, self.x_goal - x)
        angle_turn = angle_to_goal - yaw
	
        twist = Twist()
        
        # PD control
        derivative = distance - self.prev_distance
        derivative_angle = angle_turn - self.prev_angle
        control = self.Kp * distance + self.Kd * derivative
        self.prev_distance = distance
        self.prev_angle = angle_turn
       
        twist.linear.x = min(control, 0.5)  # Limit max speed
        twist.angular.z = self.Kp_angle*(angle_turn) + self.Kd_angle*(derivative_angle)

        if self.i < 3999:
            self.time_now = time.time()
            if self.time_now - self.time_last > self.frequency:
                self.time_last = time.time()
                self.i += 1
                self.x_goal = self.goal[0][self.i]
                self.y_goal = self.goal[1][self.i]

        if distance < self.arrival_threshold:
            # Stop the robot by publishing zero velocities
            twist = Twist()
            self.publisher_.publish(twist)
            self.get_logger().info(f"Goal reached: x={self.x_goal}, y={self.y_goal}")
            print(f"I am here")
            return 
        twist.linear.x = distance
        self.publisher_.publish(twist)

        self.actual_x.append(x)
        self.actual_y.append(y)
        print(f"Current Location - x: {x}, y: {y}, Target - x: {self.x_goal}, y: {self.y_goal}")
        print(f"Current Velocity -Vel: {twist.linear.x}")
        
    def plot_trajectories(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.goal[0], self.goal[1], 'r--', label='Desired Trajectory')
        plt.plot(self.actual_x, self.actual_y, 'b-', label='Actual Trajectory')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.title('Desired vs Actual Trajectory')
        plt.legend()
        plt.show()
        
'''
        if distance < 0.05 and self.i < 99:
        	self.i += 1
        	self.x_goal = self.goal[0][self.i]
        	self.y_goal = self.goal[1][self.i]
'''

        
'''        
        te = math.acos(msg.pose.pose.orientation.w) * 2
        if y < 0:
            angle = math.atan2(y - y_start, x - x_start) * (180 / math.pi)
            if -1 * angle < 0:
                base_cmd.angular.z = -1.5 * ((180 - angle) - te * (180 / math.pi))
            else:
                base_cmd.angular.z = -1.5 * (-angle - te * (180 / math.pi))
        elif y > 0:
            angle = math.atan2(y - y_start, x - x_start) * (180 / math.pi)
            if angle < 0:
                base_cmd.angular.z = 0.5 * (abs(angle) - te * (180 / math.pi)) + 0.0035 * error_sum + 0.5 *error_change
                base_cmd.linear.z = abs(angle) - te * (180 / math.pi)
            else:
                base_cmd.angular.z = 0.5 * (angle - te * (180 / math.pi)) + 0.0005 * error_sum + 0.5 * error_change
                base_cmd.linear.z = angle - te * (180 / math.pi)
        else:
            if x <= 0:
                base_cmd.angular.z = -1.5 * (179 - te * (180 / math.pi))
'''        
        
    
def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscribe()
    rclpy.spin_once(subscriber)
    rclpy.spin_once(subscriber)
    #x_traj = np.linspace(subscriber.x, -20, 100, endpoint=True)
    #y_traj = np.linspace(subscriber.y, -20, 100, endpoint=True)
    #y_traj = 3*np.sin(0.5 * (x_traj-x_traj[0]))
    step = -0.0015 # 100hz/0.01
    x_traj = subscriber.x + np.arange(0, 4000) * step
    # y_traj = subscriber.y + np.arange(0, 4000) * step
    y_traj = 1.5*np.sin(0.5 * (x_traj-x_traj[0])) - subscriber.y
    
    plt.plot(x_traj,y_traj)
    plt.show()
    # y_traj = sin(w * x)
    subscriber.destroy_node()
    pos = [x_traj,y_traj]
    controller = CircularTrajectoryController(pos)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
    if controller.actual_x:  # Ensure there is data to plot
        controller.plot_trajectories()

if __name__ == '__main__':
    main()