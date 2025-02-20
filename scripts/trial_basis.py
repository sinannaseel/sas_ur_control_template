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

from dqrobotics.interfaces.coppeliasim import DQ_CoppeliaSimInterfaceZMQ
from dqrobotics.robot_control import DQ_ClassicQPController

######################################

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver import RobotDriverClient

from sas_core import Clock, Statistics


vi = DQ_CoppeliaSimInterfaceZMQ()

def main(args=None):
    try:
        rclcpp_init()
        node = rclcpp_Node("sas_robot_driver_ur_joint_space_example_node_cpp")

        # 10 ms clock
        clock = Clock(0.01)
        clock.init()

        # Initialize the RobotDriverClient
        rdi = RobotDriverClient(node, 'ur_composed')
        vi.connect("192.168.1.211", 23000, 500, 1)

        # Wait for RobotDriverClient to be enabled
        while not rdi.is_enabled():
            rclcpp_spin_some(node)
            time.sleep(0.1)

        # Get topic information
        print(f"topic prefix = {rdi.get_topic_prefix()}") 

        # Read the values sent by the RobotDriverServer
        joint_positions = rdi.get_joint_positions()
        print(f"joint positions = {joint_positions}")

        ################## editing the code from here ##########################

        ################################### Initializing main robot
        # Define DH parameters
        UR3e_DH_theta = np.array([np.pi, 0, 0, 0, 0, np.pi/2])
        UR3e_DH_d = np.array([0, 0, 0, 0.11188, 0.08535, 0.0921])
        UR3e_DH_a = np.array([0, -0.24335, -0.2132 - 0.00435, 0, 0, 0])
        UR3e_DH_alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, -np.pi/2])

        # Define joint types as revolute
        UR3e_DH_type = np.full((1, 6), DQ_JointType.REVOLUTE, dtype=float)

        # Combine into a single matrix
        UR3e_DH_matrix = np.vstack([UR3e_DH_theta, UR3e_DH_d, UR3e_DH_a, UR3e_DH_alpha, UR3e_DH_type])

        ### set base reference


        ### controller defination
        qp_solver = 
        translation_controller = DQ_ClassicQPController('ur_composed',)



        dream = vi.get_object_pose('trial',"trial")



        print(f"naseels_pose={dream}")

        ################# from before ##########################################

        # For some iterations. Note that this can be stopped with CTRL+C.
        for i in range(0, 5000):
            clock.update_and_sleep()

            # Move the joints
            target_joint_positions = joint_positions + deg2rad([30.0 * sin(i / (50.0 * pi))] * 6)
            # print(target_joint_positions)
            rdi.send_target_joint_positions(target_joint_positions)

            rclcpp_spin_some(node)


        ################## editing the code till here ##########################

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
