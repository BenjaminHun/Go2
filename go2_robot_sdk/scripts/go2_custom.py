from go2_robot_sdk.scripts.go2_func import gen_mov_command


def generate_custom_commands(robot_cmd_vel, robot_num):
    # Example: Set custom commands for all robots
    custom_x = 0.0  # Custom linear velocity
    custom_z = 0.1  # Custom angular velocity
    robot_cmd_vel[robot_num] = gen_mov_command(custom_x, 0.0, custom_z)