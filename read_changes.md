
# Changes in the code : 



# Understanding te code :


1. real_robot_launch.py file is mostly a singular file with all config set up in it.

---

1. compose_with_coppeliasim_launch.py : configure the IP and system configuration
2. dummy_robot_launch file.py : configure the robot specific settings
3. dummy_with_coppeliasim_launch.py : {calls 1 & 2} calls two launch file(above two) to make the simulated robot move in coppeliasim
4. dummy_move_in_coppeliasim_example_cpp_launch.py: {Calls 3 & 10} calls the file that calls the other two + example script [cpp example, for py example dummy_move_in_coppeliasim_example_launch.py]

10. joint_interface_example.cpp : example movement script
11. joint_interface_example.py : example movement script for python


