#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 16 11:14:54 2024

@author: pablo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.joint_state = JointState()
        self.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']  # Asegúrate de usar los nombres correctos
        self.joint_state.position = [0.5, 0.2, -0.5, 1.0, -0.3, 0.5]  # Valores deseados para las posiciones de los joints

    def timer_callback(self):
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f'Publicando posiciones de joints: {self.joint_state.position}')


if __name__ == '__main__':
    rclpy.init()
    joint_publisher = JointPublisher()

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()
