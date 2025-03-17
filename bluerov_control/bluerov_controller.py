import numpy as np
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from bluerov_control.PIDController import PIDController

class BlueROVController(Node):
    def __init__(self):
        super().__init__('bluerov_controller')

        self.get_logger().debug("calculating pid")

        self.ns = self.get_namespace()
        self.get_logger().info(">>>>>>>>>>> namespace =" + self.ns)

        # create publishers

        # create subscribers
        # subscribe to filtered states and trajectory
        self.odom_sub = self.create_subscription(Odometry, 'odom_topic_name', self.odom_callback, 10)
        self.cmd_pose_sub = self.create_subscription(Pose, 'cmd_pose_topic_name', self.pose_callback, 10)
        
        # timer variables
        control_rate = 20
        self.control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(self.control_period, self.control_callback)

        ################## CONTROL ##################
        self.cmd_pose = Pose()
        self.odom = Odometry()

        # control switch

        # bouyancy compensation

        # PID initialization for depth and yaw
        self.pid_depth = PIDController(type='linear')
        self.pid_yaw = PIDController(type='angular')

        self.config = {}
        self._declare_and_fill_map('k_p_depth', 4.0, "K P of depth", self.config)
        self._declare_and_fill_map('k_i_depth', 0.5, "K I of depth", self.config)
        self._declare_and_fill_map('k_d_depth', 0.0, "K D of depth", self.config)

        self._declare_and_fill_map('k_p_yaw', 0.5, "K P of yaw", self.config)
        self._declare_and_fill_map('k_i_yaw', 0.0, "K I of yaw", self.config)
        self._declare_and_fill_map('k_d_yaw', 0.5, "K D of yaw", self.config)

        self.add_on_set_parameters_callback(self.callback_params)
        
        # set up service


    def control_switch_callback(self, msg):
        """
        Enable or disable the controller
        """
        pass

    def pose_callback(self, msg):
        """
        Pose callback function
        """
        pass

    def odom_callback(self, msg):
        """
        Odometry callback function
        """       
        pass

    def control_callback(self, msg):
        """
        Control callback function
        """
        pass

    def depth_control(self):
        """
        Depth control using PI controller
        """
        pass

    def yaw_control(self):
        """
        Yaw control using PID controller
        """
        pass
    
    def update_control_param(self):
        """
        Update the control parameters
        """
        # self.pid_depth.reconfig_param(self.config['k_p_depth'], self.config['k_i_depth'], self.config['k_d_depth'])
        # self.pid_yaw.reconfig_param(self.config['k_p_yaw'], self.config['k_i_yaw'], self.config['k_d_yaw'])
        pass

    def callback_params(self, params):
        """
        Callback function for dynamic reconfigure
        """
        # for param in params:
        #     self.config[param.name] = param.value
        # self.update_control_param()
        # return SetParametersResult(successful=True)
        pass

    def _declare_and_fill_map(self, key, default_value, description, map):
        """
        Declare a parameter and fill the map
        """
        # param = self.declare_parameter(
        #     key, default_value, ParameterDescriptor(description=description))
        # map[key] = param.value
        pass

    def declare_and_set_params(self):
        """
        Declare and set the parameters
        """
        # self.config = {}
        # self._declare_and_fill_map('k_p_depth', 4.0, "K P of depth", self.config)
        # self._declare_and_fill_map('k_i_depth', 0.0, "K I of depth", self.config)
        # self._declare_and_fill_map('k_d_depth', 0.0, "K D of depth", self.config)

        # self._declare_and_fill_map('k_p_yaw', 0.5, "K P of yaw", self.config)
        # self._declare_and_fill_map('k_i_yaw', 0.0, "K I of yaw", self.config)
        # self._declare_and_fill_map('k_d_yaw', 0.5, "K D of yaw", self.config)

        # self.update_control_param()
        pass


    ### HELPER FUNCTIONS ###
    def euler_from_quaternion(self, x, y, z, w):
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

        return roll_x, pitch_y, yaw_z  # in radians
    
    def eulerang(self, phi, theta, psi):
        """  
        Generate the transformation 6x6 matrix J 
        and 3x3 matrix of j_11 and j_22
        which corresponds to eq 2.40 on p.26 (Fossen 2011)
        """

        cphi = math.cos(phi)
        sphi = math.sin(phi)
        cth = math.cos(theta)
        sth = math.sin(theta)
        cpsi = math.cos(psi)
        spsi = math.sin(psi)

        if cth == 0:
            return -1

        # corresponds to eq 2.18 on p.22 (Fossen 2011)
        r_zyx = np.array([[cpsi*cth,  -spsi*cphi+cpsi*sth*sphi,  spsi*sphi+cpsi*cphi*sth],
                          [spsi*cth,  cpsi*cphi+sphi*sth *
                              spsi,   -cpsi*sphi+sth*spsi*cphi],
                          [-sth,      cth*sphi,                  cth*cphi]])

        # corresponds to eq 2.28 on p.25 (Fossen 2011)
        t_zyx = np.array([[1,  sphi*sth/cth,  cphi*sth/cth],
                          [0,  cphi,          -sphi],
                          [0,  sphi/cth,      cphi/cth]])

        # corresponds to eq 2.40 on p.26 (Fossen 2011)
        j_1 = np.concatenate((r_zyx, np.zeros((3, 3))), axis=1)
        j_2 = np.concatenate((np.zeros((3, 3)), t_zyx), axis=1)
        j = np.concatenate((j_1, j_2), axis=0)

        return j, r_zyx, t_zyx
    
def main(args=None):
    rclpy.init(args=args)
    node = BlueROVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

