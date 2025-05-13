import numpy as np
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from bluerov_control.PIDController import PIDController
from bluerov_control.bluerov_helper import Helper

class BlueROVController(Node):
    def __init__(self):
        super().__init__('bluerov_controller')

        self.get_logger().debug("calculating pid")

        self.ns = self.get_namespace()
        self.get_logger().info(">>>>>>>>>>> namespace =" + self.ns)

        # create publishers
        self.pub_depth_pwm = self.create_publisher(Float64, '/control/depth_pwm', 10)
        self.pub_yaw_pwm = self.create_publisher(Float64, '/control/yaw_pwm', 10)
        self.pub_surge_pwm = self.create_publisher(Float64, '/control/surge_pwm', 10)
        self.pub_sway_pwm = self.create_publisher(Float64, '/control/sway_pwm', 10)

        # create subscribers
        self.subimu = self.create_subscription(Imu, "imu/data", self.imu_callback)
        self.subrel_alt = self.create_subscription(Float64, "global_position/rel_alt", self.depth_callback)

        # TODO: subscribe to ping
        
        # timer variables
        control_rate = 20
        self.control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(self.control_period, self.control_callback)

        ################## CONTROL ##################

        # control switch
        self.is_control_on = False

        # bouyancy compensation

        # PID initialization for depth and yaw
        self.pid_depth = PIDController(type='linear')
        self.pid_yaw = PIDController(type='angular')
        self.pid_surge = PIDController(type='linear')
        self.pid_sway = PIDController(type='linear')

        self.config = {}

        self._declare_and_fill_map('k_p_surge', 4.0, "K P of surge", self.config)
        self._declare_and_fill_map('k_i_surge', 0.5, "K I of surge", self.config)
        self._declare_and_fill_map('k_d_surge', 0.0, "K D of surge", self.config)

        self._declare_and_fill_map('k_p_sway', 4.0, "K P of sway", self.config)
        self._declare_and_fill_map('k_i_sway', 0.5, "K I of sway", self.config)
        self._declare_and_fill_map('k_d_sway', 0.0, "K D of sway", self.config)

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
        self.pid_surge.reset_control()
        self.pid_sway.reset_control()

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
    
    def imu_callback(self, msg):
        # Extract yaw from quaternion
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = yaw

    def depth_callback(self, msg):
        self.current_depth = msg.data


    def control_callback(self, msg):
        """
        Control callback function
        """
        if not self.is_control_on:
            return

        current_time = self.get_clock().now().nanoseconds * 1e-9

        depth_pwm = self.depth_control(current_time)
        yaw_pwm = self.yaw_control(current_time)

        self.pub_depth_pwm.publish(Float64(data=depth_pwm))
        self.pub_yaw_pwm.publish(Float64(data=yaw_pwm))

    def depth_control(self, current_time):
        """
        Depth control using PI controller
        """
        err, int_err, diff_err = self.pid_depth.calculate_pid(self.target_depth, self.current_depth, current_time)
        control = self.pid_depth.calculate_pid(err, int_err, diff_err)
        self.get_logger().info(f"Depth Control: {control}")
        return control

    def yaw_control(self):
        """
        Yaw control using PID controller
        """
        err, int_err, diff_err = self.pid_yaw.calculate_pid(self.target_yaw, self.current_yaw)
        control = self.pid_yaw.calculate_pid(err, int_err, diff_err)
        self.get_logger().info(f"Yaw Control: {control}")
        return control
    
    def update_control_param(self):
        """
        Update the control parameters
        """
        self.pid_depth.reconfig_param(self.config['k_p_depth'], self.config['k_i_depth'], self.config['k_d_depth'])
        self.pid_yaw.reconfig_param(self.config['k_p_yaw'], self.config['k_i_yaw'], self.config['k_d_yaw'])
        self.pid_surge.reconfig_param(self.config['k_p_surge'], self.config['k_i_surge'], self.config['k_d_surge'])
        self.pid_sway.reconfig_param(self.config['k_p_sway'], self.config['k_i_sway'], self.config['k_d_sway'])
        

    def callback_params(self, params):
        for param in params:
            self.config[param.name] = param.value
        self.update_control_param()
        return SetParametersResult(successful=True)

    def update_control_param(self):
        self.pid_depth.reconfig_param(
            self.config['k_p_depth'], self.config['k_i_depth'], self.config['k_d_depth'])
        self.pid_yaw.reconfig_param(
            self.config['k_p_yaw'], self.config['k_i_yaw'], self.config['k_d_yaw'])

    def declare_and_set_params(self):
        self._declare_and_fill_map('k_p_depth', 4.0, "K P of depth", self.config)
        self._declare_and_fill_map('k_i_depth', 0.0, "K I of depth", self.config)
        self._declare_and_fill_map('k_d_depth', 0.0, "K D of depth", self.config)
        self._declare_and_fill_map('k_p_yaw', 1.0, "K P of yaw", self.config)
        self._declare_and_fill_map('k_i_yaw', 0.0, "K I of yaw", self.config)
        self._declare_and_fill_map('k_d_yaw', 0.0, "K D of yaw", self.config)
        self.update_control_param()

    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(
            key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value

    
def main(args=None):
    rclpy.init(args=args)
    node = BlueROVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

