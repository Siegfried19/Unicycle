"""
logic:
1. Get initial position through single time subscription
2. Calulate the trajectory with initial position
3. Controller
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from matplotlib.ticker import MultipleLocator
import math
import time
import numpy as np
import matplotlib.pyplot as plt


class TrajectoryGenerator(Node):
    def __init__(self, step):
        super().__init__('get_initial_position')
        self.x_traj = []
        self.y_traj = []
        self.step = step
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_traj = x + np.cos(np.arange(0, 4000) * self.step * 0.5 * np.pi)
        self.y_traj = y + np.sin(np.arange(0, 4000) * self.step * 0.5 * np.pi)

# Controller
class CircularTrajectoryController(Node):
    def __init__(self, goal):
        super().__init__('circular_trajectory_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription_odom
        
        # Target trajectory
        self.goal = goal
        self.x_tar = goal[0][0]
        self.y_tar = goal[1][0]
        self.i = 0

        # Real trajectory to plot
        self.actual_x = []
        self.actual_y = []
        self.flag = 1

        # Control period
        self.time_now = time.time()
        self.time_last = time.time()

        # PD parameter for distance and degree control
        self.ang_kp = 1.4
        self.ang_kd = 0.1
        self.dis_kp = 1
        self.dis_kd = 0
        self.ang_prev_error = 0.0
        self.dis_prev_error = 0.0
        self.control_time = 0.01
        self.threshold = 0.1

    def plot_trajectories(self):
        fig, ax = plt.subplots()
        ax = plot_setup(ax)
        ax.plot(self.goal[0], self.goal[1], label='Desired Trajectory',color = 'blue', linewidth = 1)
        ax.plot(self.actual_x, self.actual_y, label='Actual Trajectory',color = 'red', linewidth = 1)
        ax.legend()
        plt.show() 

    def odom_callback(self, msg):
        # Current status
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        # Plot data
        self.actual_x.append(x)
        self.actual_y.append(y)

        # Calculate the error
        dis_error = math.sqrt(pow(self.x_tar - x, 2) + pow(self.y_tar - y, 2))
        ang_tar = math.atan2((self.y_tar - y), (self.x_tar - x))
        ang_error = ang_tar - yaw
        if ang_error < -math.pi:
            ang_error = ang_error + 2 * math.pi
        elif ang_error > math.pi:
            ang_error = ang_error - 2 * math.pi

        # Control output
        ang_derivative = ang_error - self.ang_prev_error
        ang_control = self.ang_kp * ang_error + self.ang_kd * ang_derivative
        self.ang_prev_error = ang_error

        dis_derivative = dis_error - self.dis_prev_error
        dis_control = self.dis_kp * dis_error + self.dis_kd * dis_derivative
        self.dis_prev_error = dis_error

        # If update target
        if self.i < 3999:
            self.time_now = time.time()
            if self.time_now - self.time_last > self.control_time:
                # self.i += int((self.time_now - self.time_last) // self.control_time)
                self.i += 1
                self.time_last = time.time()
                self.x_tar = self.goal[0][self.i]
                self.y_tar = self.goal[1][self.i]


        # Create and publish Twist message
        if dis_error < self.threshold:
            # Publish 0 control
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            if self.i ==3999:
                self.get_logger().info(f"Goal reached: x={self.x_tar}, y={self.y_tar}")
                print(f"I am here")
                self.flag = 0
            return 
        twist = Twist()
        twist.linear.x = min(dis_control, 0.5)  
        twist.angular.z = ang_control  
        self.publisher_.publish(twist)
        

        print(f"Current Location    -   x: {x}, y: {y}")
        print(f"target Location     -   x: {self.x_tar}, y: {self.y_tar}")
        print(f"distence error      -   {dis_error}")
        print(f"degree target       -   {ang_tar}")
        print(f"degree current      -   {yaw}")
        print(f"degree error        -   {ang_error}")
        print(f"distance control    -   {dis_control}")
        print(f"degree control      -   {ang_control}")
        print(f"step                -   {self.i}")
    


# Set up the plot
def plot_setup(ax):
    ax.grid(True)
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.yaxis.set_major_locator(MultipleLocator(1))
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    return ax


def main(args=None):
    step = -0.0015
    rclpy.init(args=args)

    # Generate trajectory based on initial position
    Traj = TrajectoryGenerator(step)
    rclpy.spin_once(Traj)
    x_traj = Traj.x_traj
    y_traj = Traj.y_traj

    # # Plot the ideal trajectory
    # fig, ax = plt.subplots()
    # plot_setup(ax).plot(x_traj,y_traj)
    # plt.show()
    Traj.destroy_node()

    # Follow this trajectory
    pos = [x_traj,y_traj]
    controller = CircularTrajectoryController(pos)
    while controller.flag:
        rclpy.spin_once(controller)
    if controller.actual_x:
        controller.plot_trajectories()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
