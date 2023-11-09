#!/usr/bin/env python3

import argparse
import rospy
from PIL import Image
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class PotentialFields:
    def __init__(self,robot_name):
        rospy.init_node('potential_fields_node')
        self.pixels_array = self.obstacles("map_1.png")
        self.posicao_atual = PoseStamped()
        self.posicao_atual.header.frame_id = 'map'  # Define o frame de referência
        self.posicao_atual.pose.position.x = 5.0  # Define a posição x inicial como 5
        self.posicao_atual.pose.position.y = 5.0  # Define a posição y inicial como 5
        self.posicao_atual.pose.position.z = 0.0  # Define a posição z inicial como 0

        self.position_publisher = rospy.Publisher(f'/{robot_name}/current_position', PoseStamped, queue_size=1)
        self.twist_publisher = rospy.Publisher(f'/{robot_name}/cmd_vel', Twist, queue_size=1)

    def obstacles(self, map):

        image = Image.open(f"src/tcc/worlds/{map}")
        self.image_gray = image.convert('L')
        self.pixels_array = np.array(self.image_gray)
        self.threshold = 254
        filled_pixels = self.pixels_array < self.threshold
        return filled_pixels

    def potential_fields(self):
        print(self.posicao_atual)
        ponto_destino = (5, 5)  # Substitua com as coordenadas do ponto de destino
        vetor_atracao = ponto_destino - np.array([self.posicao_atual.pose.position.x, self.posicao_atual.pose.position.y])

        vetor_repulsao_total = np.zeros(2)  # Inicialize com vetor nulo
        constante_repulsao = 1000  # Ajuste conforme necessário

        for i in range(self.pixels_array.shape[0]):
            for j in range(self.pixels_array.shape[1]):
                if self.pixels_array[i, j]:
                    vetor_repulsao = np.array([j, i]) - np.array([self.posicao_atual.pose.position.x, self.posicao_atual.pose.position.y])
                    magnitude = constante_repulsao / np.linalg.norm(vetor_repulsao)**2
                    vetor_repulsao_total += vetor_repulsao / np.linalg.norm(vetor_repulsao) * magnitude

        vetor_resultante = vetor_atracao + vetor_repulsao_total

        self.posicao_atual.pose.position.x += vetor_resultante[0]
        self.posicao_atual.pose.position.y += vetor_resultante[1]

        # Publica a posição atual
        self.position_publisher.publish(self.posicao_atual)
        
        # Enviar comandos para o robô (exemplo hipotético)
        velocidade_linear = 0.1  # Ajuste conforme necessário
        velocidade_angular = 0.0  # Ajuste conforme necessário

        # Comande o movimento do robô (depende do seu ambiente e tipo de robô)
        # Exemplo hipotético:
        self.move_robot(velocidade_linear, velocidade_angular)

    def move_robot(self, linear, angular):
        comando = Twist()
        comando.linear.x = linear
        comando.angular.z = angular
        print("comando", comando)
        self.twist_publisher.publish(comando)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.potential_fields() 
            rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Control a human-like robot.')
    parser.add_argument('robot_name', type=str, help='Name of the robot')
    args = parser.parse_args()
    try:
        pf = PotentialFields(args.robot_name)
        pf.run()
    except rospy.ROSInterruptException:
        pass
