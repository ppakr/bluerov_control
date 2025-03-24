import numpy as np
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool

from bluerov_control.PIDController import PIDController
from bluerov_control.bluerov_helper import Helper

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
        self.is_control_on = False

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

    def reset_controller(self):
        self.pid_depth.reset_control()
        self.pid_yaw.reset_control()

    def control_switch_callback(self, request, response):
        """
        Enable or disable the controller
        """
        
        if request.data == True:
            self.reset_controller()
            response.message == "Control Enabled"
        else:
            self.reset_controller()
            response.message == "Control Disabled"

        self.is_control_on = request.data

        response.success = True

        return response


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

    
def main(args=None):
    rclpy.init(args=args)
    node = BlueROVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

