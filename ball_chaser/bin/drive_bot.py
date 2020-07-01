#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ball_chaser.srv import DriveToTarget, DriveToTargetResponse

motor_command_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

def handle_drive_request(req):
    rospy.loginfo("DriveToTargetRequest received - x:%f, z:%f", req.linear_x, req.angular_z)
    
    motor_command = Twist()
    motor_command.linear.x = req.linear_x
    motor_command.angular.z = req.angular_z

    motor_command_publisher.publish(motor_command)
    return DriveToTargetResponse("Velocities set -x: " + str(motor_command.linear.z) + ", z: " + str(motor_command.angular.z))


def drive_bot_server():
    
    rospy.init_node("drive_bot", anonymous=True)
    service = rospy.Service("/ball_chaser/command_robot", DriveToTarget, handle_drive_request)
    rospy.spin()


if __name__ =='__main__':
    drive_bot_server()