#!/usr/bin/python3
"""
# Copyright (c) 2012-2024 Murilo Marques Marinho
#
#    This file is part of sas_ur_control_template.
#
#    sas_ur_control_template is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_ur_control_template is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_ur_control_template.  If not, see <https://www.gnu.org/licenses/>.
#
# #######################################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#   Based on `joint_interface_example.py` from `sas_robot_driver_kuka`
#
# #######################################################################################
"""
import time
import numpy as np

from math import sin, pi

import numpy
from dqrobotics import *  # Despite what PyCharm might say, this is very much necessary or DQs will not be recognized
from dqrobotics.utils.DQ_Math import deg2rad

########### added by me #############

from dqrobotics.utils import DQ_Geometry
from dqrobotics.interfaces.coppeliasim import DQ_CoppeliaSimInterfaceZMQ
from dqrobotics.robot_control import DQ_ClassicQPController
from dqrobotics.solvers import DQ_QuadraticProgrammingSolver
from dqrobotics import *

###########

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver import RobotDriverClient

from sas_core import Clock, Statistics


def main(args=None):
    try:
        rclcpp_init()
        node = rclcpp_Node("sas_robot_driver_ur_joint_space_example_node_cpp")

        # 10 ms clock
        clock = Clock(0.01)
        clock.init()

        # Initialize the RobotDriverClient
        rdi = RobotDriverClient(node, 'ur_composed')

        # Wait for RobotDriverClient to be enabled
        while not rdi.is_enabled():
            rclcpp_spin_some(node)
            time.sleep(0.1)

        # Get topic information
        print(f"topic prefix = {rdi.get_topic_prefix()}")

        # Read the values sent by the RobotDriverServer
        joint_positions = rdi.get_joint_positions()
        print(f"joint positions = {joint_positions}")

        ################################ trying to code boss

        # trying to connect
        vi = DQ_CoppeliaSimInterfaceZMQ()
        vi.connect("192.168.0.187", 23000, 500, 1)

        # Define joint names
        jointnames = ['UR3_joint1', 'UR3_joint2', 'UR3_joint3', 
                    'UR3_joint4', 'UR3_joint5', 'UR3_joint6']
        
        # Define DH parameters
        UR3e_DH_theta = np.array([np.pi, 0, 0, 0, 0, np.pi/2])
        UR3e_DH_d = np.array([0, 0, 0, 0.11188, 0.08535, 0.0921])
        UR3e_DH_a = np.array([0, -0.24335, -0.2132 - 0.00435, 0, 0, 0])
        UR3e_DH_alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, -np.pi/2])
        UR3e_DH_type = np.full((1, 6), DQ_JointType.REVOLUTE, dtype=float)
        
        # Stack DH parameters into a 5x6 matrix
        UR3e_DH_matrix = np.vstack([UR3e_DH_theta, UR3e_DH_d, UR3e_DH_a, UR3e_DH_alpha, UR3e_DH_type])

        robot = DQ_SerialManipulatorDH(UR3e_DH_matrix)
        n = robot.get_dim_configuration_space()  # Configuration space dimension

        # Define joint limits (adjust if necessary)
        qmin = np.array([-np.pi, -np.pi, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2])
        qmax = np.array([ np.pi,  np.pi,  np.pi/2,  np.pi,  np.pi/2,  np.pi/2])

        # Set base reference frame from simulation object pose (assumes object name 'UR3_joint1')
        xbase = vi.get_object_pose('UR3_joint1')
        robot.set_reference_frame(xbase)
        robot.set_base_frame(xbase)
        vi.set_object_pose('base', xbase)
        
        # =============================================================================
        # ======= Controller Definition ===============================================
        # =============================================================================

        qp_solver = DQ_QuadraticProgrammingSolver()
        translation_controller = DQ_ClassicQPController('ur_composed',)
        translation_controller = DQ_ClassicQPController(robot, qp_solver)
        translation_controller.set_gain(10)      # Controller gain
        translation_controller.set_damping(1)      # Damping factor
        translation_controller.set_control_objective(ControlObjective.Translation)

        eta_d = 1  # VFI gain (if applicable)

        # =============================================================================
        # ======= Desired Data & Pose =================================================
        # =============================================================================


        dream = vi.get_object_pose('trial',"trial")
        # Get the goal and start poses from the simulation environment
        goal_pose = vi.get_object_pose('trial','trial')
        goal_translation = translation(goal_pose)
        #Start = translation(vi.get_object_pose('/Start_position'))

        # =============================================================================
        # ======= Simulation Parameters ===============================================
        # =============================================================================

        tau = 0.01      # Sampling time [s]
        final_time = 5  # Total simulation time ``[s]
        latency_counter = 0
        j = 1

        # Initialize arrays to store error values (if needed)
        error_signal_values = []
        error_special_values = []


        print("Naseel is great")

        # ======================================================the original code starts here


        # For some iterations. Note that this can be stopped with CTRL+C.
        for i in range(0, 5000):
            clock.update_and_sleep()

            # Move the joints
            target_joint_positions = joint_positions + deg2rad([10.0 * sin(i / (50.0 * pi))] * 6)
            # print(target_joint_positions)
            rdi.send_target_joint_positions(target_joint_positions)

            rclcpp_spin_some(node)

        # Statistics
        print("Statistics for the entire loop")
        print("  Mean computation time: {}".format(clock.get_statistics(
            Statistics.Mean, Clock.TimeType.Computational)
        ))
        print("  Mean idle time: {}".format(clock.get_statistics(
            Statistics.Mean, Clock.TimeType.Idle)
        ))
        print("  Mean effective thread sampling time: {}".format(clock.get_statistics(
            Statistics.Mean, Clock.TimeType.EffectiveSampling)
        ))

        rclcpp_shutdown()

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print("Unhandled excepts", e)


if __name__ == '__main__':
    main()
