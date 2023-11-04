import rospy
from geometry_msgs.msg import Twist
import numpy as np

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.rad = radius

class PotentialField:
    def __init__(self, obstacles):
        self.min_vel = 2
        self.max_vel = 40
        self.map_dim = self.mapw, self.maph = (10, 10)
        self.field = np.zeros((self.mapw, self.maph, 2))
        self.goal_pose = self.gx, self.gy = (8, 8)
        self.goal_radius = 0.5
        self.goal_field = None
        self.obstacles = obstacles
        self.obstacle_field = dict((i, np.array(0)) for i in obstacles)
        self.updated = False
        self.virtual = False
        self.fcf = 5

    def start(self):
        self.goal_field = self.attract_goal(self.goal_radius)
        self.field = self.goal_field
        self.updated = False
        for obs in self.obstacles:
            if not obs in self.obstacle_field.keys():
                self.obstacle_field[obs] = self.repel_obstacle(obs)
            self.field += self.obstacle_field[obs]
        self.clampField(25)

    def draw(self):
        print(self.field)

    def attract_goal(self, radius):
        target_pos = self.goal_pose
        x = np.linspace(0, self.mapw - 1, self.mapw)
        y = np.linspace(0, self.maph - 1, self.maph)
        meshgrid = np.meshgrid(x, y, sparse=False, indexing='ij')
        meshgridX = target_pos[0] - meshgrid[0]
        meshgridY = target_pos[1] - meshgrid[1]
        field = np.zeros((self.mapw, self.maph, 2))
        field[:, :, 0] = meshgridX
        field[:, :, 1] = meshgridY
        magnitudeField = np.sqrt((field[:, :, 0] ** 2 + field[:, :, 1] ** 2)*2)
        magnitudeField = np.clip(magnitudeField, 0.0000001, np.inf)
        normalField = np.zeros((self.mapw, self.maph, 2))
        normalField[:, :, 0] = field[:, :, 0] / magnitudeField
        normalField[:, :, 1] = field[:, :, 1] / magnitudeField
        magnitudeField[np.where(magnitudeField <= self.goal_radius)] = cvtRange(magnitudeField[np.where(magnitudeField <= self.goal_radius)], 0, radius, self.max_vel, self.min_vel)
        magnitudeField[np.where(magnitudeField > radius)] = 15
        field[:, :, 0] = normalField[:, :, 0] * magnitudeField
        field[:, :, 1] = normalField[:, :, 1] * magnitudeField
        return field

    def repel_obstacle(self, obs):
        repulsePos = (obs.x, obs.y)
        x = np.linspace(0, self.mapw - 1, self.mapw)
        y = np.linspace(0, self.maph - 1, self.maph)
        meshgrid = np.meshgrid(x, y, sparse=False, indexing='ij')
        meshgridX = meshgrid[0] - repulsePos[0]
        meshgridY = meshgrid[1] - repulsePos[1]
        field = np.zeros((self.mapw, self.maph, 2))
        field[:, :, 0] = meshgridX
        field[:, :, 1] = meshgridY
        magnitudeField = np.sqrt((field[:, :, 0] ** 2 + field[:, :, 1] ** 2))
        magnitudeField = np.clip(magnitudeField, 0.0000001, np.inf)
        normalField = np.zeros((self.mapw, self.maph, 2))
        normalField[:, :, 0] = field[:, :, 0] / magnitudeField
        normalField[:, :, 1] = field[:, :, 1] / magnitudeField
        filter_ = np.where(magnitudeField <= obs.rad*2.5)
        if len(filter_) != 0:
            magnitudeField[filter_] = cvtRange(magnitudeField[filter_], 0, obs.rad*2.5, self.max_vel, self.min_vel)
        filter_ = np.where(magnitudeField > obs.rad*2.5)
        if len(filter_) != 0:
            magnitudeField[filter_] = 0
        field[:, :, 0] = normalField[:, :, 0] * magnitudeField
        field[:, :, 1] = normalField[:, :, 1] * magnitudeField
        return field

    def clampField(self, maxVel):
        magnitudeField = np.sqrt(self.field[:, :, 0] ** 2 + self.field[:, :, 1] ** 2)
        magnitudeField = np.clip(magnitudeField, 0.000001, np.inf)
        normalField = np.zeros((self.mapw, self.maph, 2))
        normalField[:, :, 0] = self.field[:, :, 0] / magnitudeField
        normalField[:, :, 1] = self.field[:, :, 1] / magnitudeField
        magnitudeField = np.clip(magnitudeField, 0, maxVel)
        self.field[:, :, 0] = normalField[:, :, 0] * magnitudeField
        self.field[:, :, 1] = normalField[:, :, 1] * magnitudeField

rospy.init_node('potential_field')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

def move_robot(linear, angular):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    cmd_vel_pub.publish(msg)

if __name__ == '__main__':
    obstacles = [Obstacle(4.0, 4.0, 0.5), Obstacle(6.0, 6.0, 0.5)]
    potential_field = PotentialField(obstacles)

    while not rospy.is_shutdown():
        repulsion_force = potential_field.repel_obstacle(5.0, 5.0)
        linear_velocity = 0.1  # Ajuste conforme necessário
        angular_velocity = 0.0  # Ajuste conforme necessário

        move_robot(linear_velocity, angular_velocity)

        rate.sleep()
