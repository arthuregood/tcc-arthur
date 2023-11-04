#!/usr/bin/env python3

import argparse
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

feedback = Odometry()

def subCallback(msg):
    #rospy.loginfo(f"msg {msg}")
    global feedback
    feedback = msg

def subCallbackSensor(data, pub):
    ranges = data.ranges

    min_distance = min(ranges)
    if min_distance < 0.5: 
        # robo para e vira para a direita

        # TODO: nao está parando, embora a lógica do sensor e if estejam funcionais
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        pub.publish(stop_msg)

        move_right_msg = Twist()
        move_right_msg.linear.x = 0.5
        move_right_msg.angular.z = 0.0
        pub.publish(move_right_msg)


def controlHuman(robot_name, robotx, roboty):
    rospy.init_node(f'control_{robot_name}', anonymous=True)
    pub = rospy.Publisher(f'/{robot_name}/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber(f'/{robot_name}/base_pose_ground_truth', Odometry, subCallback)
    rospy.Subscriber(f'/{robot_name}/base_scan', LaserScan, subCallbackSensor, callback_args=pub)

    xdesejado = float(robotx)
    ydesejado = float(roboty)

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

    while abs(dist) > tolerance:

        actual_time = rospy.get_time()
        dt = actual_time - last_time

        # checagem se dt não é zero
        if abs(dt) < 1e-6:
            dt = 1e-6

        dist = math.sqrt(math.pow(xdesejado-feedback.pose.pose.position.x, 2) +
                        math.pow(ydesejado-feedback.pose.pose.position.y, 2))
        errorInt = errorInt + (dist * dt)
        errorDer = (dist - lastError) / dt
        msg.linear.x = Kp*dist + Ki * errorInt + Kd * errorDer

        #rospy.loginfo("x %f, y %f, erro %f", feedback.pose.pose.position.x, feedback.pose.pose.position.y, dist)
        msg.angular.z = Korie * \
            (math.atan2(ydesejado - feedback.pose.pose.position.y, xdesejado - feedback.pose.pose.position.x) - euler_from_quaternion([feedback.pose.pose.orientation.x, feedback.pose.pose.orientation.y, feedback.pose.pose.orientation.z, feedback.pose.pose.orientation.w])[2])

        pub.publish(msg)
        rate.sleep()

        last_time = actual_time
        lastError = dist

    rospy.loginfo("Trajeto finalizado")
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Control a human-like robot.')
    parser.add_argument('robot_name', type=str, help='Name of the robot')
    parser.add_argument('robotx', type=float, help='X coordinate of the goal')
    parser.add_argument('roboty', type=float, help='Y coordinate of the goal')
    args = parser.parse_args()

    try:
        controlHuman(args.robot_name, args.robotx, args.roboty)
    except rospy.ROSInterruptException:
        pass
