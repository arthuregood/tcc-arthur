#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
from PIL import Image
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Quaternion

class PotentialFields:
    def __init__(self):
        rospy.init_node('potential_fields_node')
        self.pixels_array = self.obstacles("map_1.png")
        self.posicao_atual = PoseStamped()
        self.posicao_atual.header.frame_id = 'map'  
        self.posicao_atual.pose.position.x = 5.0  
        self.posicao_atual.pose.position.y = 5.0  
        self.posicao_atual.pose.position.z = 0.0 

        self.orientacao_atual = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.position_publisher = rospy.Publisher('/current_position', PoseStamped, queue_size=1)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_publisher = rospy.Publisher("/odom", PoseStamped, self.callback_pose, queue_size=1)

    def callback_pose(self, msg):
        print("msg", msg)
        self.orientacao_atual = msg.orientation

    def obstacles(self, map):
        image = Image.open(f"src/tcc/worlds/{map}")
        self.image_gray = image.convert('L')
        self.pixels_array = np.array(self.image_gray)
        self.threshold = 254
        filled_pixels = self.pixels_array < self.threshold
        return filled_pixels

    def potential_fields(self):
        ponto_destino = (5, 5)  
        vetor_atracao = ponto_destino - np.array([self.posicao_atual.pose.position.x, self.posicao_atual.pose.position.y])

        vetor_repulsao_total = np.zeros(2)  
        constante_repulsao = 1000  

        distancia_minima = float('inf')

        for i in range(self.pixels_array.shape[0]):
            for j in range(self.pixels_array.shape[1]):
                if self.pixels_array[i, j]:

                    # Calcula a distância entre o robô e o obstáculo
                    distancia = np.linalg.norm(np.array([j, i]) - np.array([self.posicao_atual.pose.position.x, self.posicao_atual.pose.position.y]))

                    if distancia < distancia_minima:
                        distancia_minima = distancia
                        vetor_repulsao = np.array([j, i]) - np.array([self.posicao_atual.pose.position.x, self.posicao_atual.pose.position.y])
                        magnitude = constante_repulsao / distancia**2
                        vetor_repulsao_total = vetor_repulsao / distancia * magnitude


        vetor_resultante = vetor_atracao + vetor_repulsao_total

        self.posicao_atual.pose.position.x += vetor_resultante[0]
        self.posicao_atual.pose.position.y += vetor_resultante[1]

        self.position_publisher.publish(self.posicao_atual)
        
        velocidade_linear = np.linalg.norm(vetor_resultante)

        # Calcular a orientação desejada e a velocidade angular
        orientacao_desejada = np.arctan2(vetor_resultante[1], vetor_resultante[0])

        orientacao_atual = self.posicao_atual.pose.orientation
        rpy = euler_from_quaternion([orientacao_atual.x, orientacao_atual.y, orientacao_atual.z, orientacao_atual.w])
        orientacao_atual_rpy = rpy[2]

        velocidade_angular = orientacao_desejada - orientacao_atual_rpy

        self.move_robot(np.linalg.norm(vetor_resultante), velocidade_angular)

    def move_robot(self, linear, angular):
        comando = Twist()
        comando.linear.x = linear
        comando.angular.z = angular
        self.twist_publisher.publish(comando)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.potential_fields() 
            rate.sleep()

if __name__ == '__main__':
    try:
        pf = PotentialFields()
        pf.run()
    except rospy.ROSInterruptException:
        pass
