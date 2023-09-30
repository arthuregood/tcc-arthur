#!/usr/bin/env python3
import math
import os
import sys
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


feedback = Pose()


def subCallback(msg):
    feedback.x = msg.x
    feedback.y = msg.y
    feedback.theta = msg.theta


def controlX():
    rospy.init_node('controlX', anonymous=True)
    os.system("rosservice call /reset")
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, subCallback)

    xdesejado = float(input("digite a posicao x: "))
    ydesejado = float(input("digite a posicao y: "))

    dist = 99.0
    Kp = 1.5
    Ki = 0.000005
    Kd = 0.000005
    errorInt = 0
    errorDer = 0
    tolerance = 0.05
    Korie = 4.0
    lastError = 0

    msg = Twist()
    rate = rospy.Rate(10)
    pub.publish(msg)

    last_time = rospy.get_time()
    print(last_time)

    while abs(dist) > tolerance:

        actual_time = rospy.get_time()
        dt = actual_time - last_time

        dist = math.sqrt(math.pow(xdesejado-feedback.x, 2) +
                         math.pow(ydesejado-feedback.y, 2))
        errorInt = errorInt + (dist * dt)
        errorDer = (dist - lastError) / dt
        msg.linear.x = Kp*dist + Ki * errorInt + Kd * errorDer

        rospy.loginfo("x %f,erro %f", feedback.x, dist)
        msg.angular.z = Korie * \
            (math.atan2(ydesejado - feedback.y, xdesejado - feedback.x)-feedback.theta)

        pose_atual = rospy.wait_for_message('/turtle1/pose', Pose)
        pub.publish(msg)
        rate.sleep()

        last_time = actual_time
        lastError = dist
    rospy.loginfo("Trajeto finalizado")
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)


if __name__ == '__main__':
    try:
        controlX()
    except rospy.ROSInterruptException:
        pass
