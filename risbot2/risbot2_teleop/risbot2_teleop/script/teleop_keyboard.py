#!/usr/bin/env python3

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

# Tốc độ tối đa và bước thay đổi vận tốc
RISBOT2_MAX_LIN_VEL = 0.26
RISBOT2_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.03

msg = """
Control Your RISBOT2!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (RISBOT2 : ~ 0.26)
a/d : increase/decrease angular velocity (RISBOT2 : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

def get_key(settings):
    if os.name == 'nt':  # Windows
        import msvcrt
        return msvcrt.getch().decode('utf-8')
    else:  # Linux or MacOS
        import termios
        import tty
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel
    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -RISBOT2_MAX_LIN_VEL, RISBOT2_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -RISBOT2_MAX_ANG_VEL, RISBOT2_MAX_ANG_VEL)

def main():
    settings = None
    if os.name != 'nt':  
        import termios
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                if key == '\x03':
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_velocity = make_simple_profile(control_linear_velocity, target_linear_velocity, LIN_VEL_STEP_SIZE / 2.0)
            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(control_angular_velocity, target_angular_velocity, ANG_VEL_STEP_SIZE / 2.0)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
