#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

feedback = Odometry()

def subCallbackSensor(msg):
    rospy.loginfo("msg", msg)
    ranges = msg.ranges

    min_distance = min(ranges)
    if min_distance < 0.5: 
        rospy.loginfo(f"Obstaculo em {min_distance}m.")


def subCallback(msg):
    global feedback
    feedback = msg

def controlX(robot_name, robotx, roboty):

    # adicionar uma checagem do sensor, caso um obstáculo apareça
    # o robô deverá tomar uma ação antes de seguir o PID

    rospy.init_node(f'control_{robot_name}', anonymous=True)
    pub = rospy.Publisher(f'/{robot_name}/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber(f'/{robot_name}/base_pose_ground_truth', Odometry, subCallback)
    rospy.Subscriber(f'/{robot_name}/laser_scan', LaserScan, subCallbackSensor)

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

        # check if dt is not zero
        if abs(dt) < 1e-6:
            dt = 1e-6

        dist = math.sqrt(math.pow(xdesejado-feedback.pose.pose.position.x, 2) +
                        math.pow(ydesejado-feedback.pose.pose.position.y, 2))
        errorInt = errorInt + (dist * dt)
        errorDer = (dist - lastError) / dt
        msg.linear.x = Kp*dist + Ki * errorInt + Kd * errorDer

        rospy.loginfo("x %f, y %f, erro %f", feedback.pose.pose.position.x, feedback.pose.pose.position.y, dist)
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
    try:
        controlX("robot_0", -1, 3)

    except rospy.ROSInterruptException:
        pass