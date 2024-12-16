import sys
from ros_robot_controller_sdk import Board


board = Board()
def setServoPulse(id, pulse, use_time):
    board.pwm_servo_set_position(use_time/1000.0, [[id, pulse]])
    
