#! /usr/bin/env python

"""
random_goal.py é um programa para aprender a usar nos no python.
Este gera um par de numeros aleatorios entre 0 e 11 para a posicao
da tartaruga (X,Y) que direciona a turtlesim para uma posicao aleatoria na tela.

Autor: Luiz C M Oliveira
"""
# Import the Python library for ROS
import rospy

# Importa as mensagens do tipo Twist e Pose2D do pacote geometry_msgs
from geometry_msgs.msg import Pose2D

# Importa a biblioteca random para numeros aleatorios
import random

# criacao de um objeto para armazenar a posicao gerada para a
# tartaruga alvo
local_aleatorio = Pose2D()  # posicao alvo

def set_random_position():
    
    # Gera um número aleatório entre 0 e 11
    local_aleatorio.x = float(random.uniform(0, 11))
    local_aleatorio.y = float(random.uniform(0, 11))
    

# Exemplo de chamada de configuração
if __name__ == "__main__":
        
    # Inicializa o topico go_to_x - conecta ao roscore
    rospy.init_node('random_goal_node')

    
    # Set a publish rate of 0.1 Hz - 1 vez a cada 10 segundos
    rate = rospy.Rate(0.25)

    # Registrar o nó como um publicador
    pub_alvo = rospy.Publisher('local_aleatorio', Pose2D, queue_size=10)

    rospy.sleep(1)  # Aguardar 1 segundo antes de iniciar o loop

    # rospy.loginfo("Publishing new random goal...")
    
    # Loop que sera executado ate o encerramento
    # da execucao do programa
    while not rospy.is_shutdown():
        
        # chama funcao para gerar uma posicao aleatoria
        set_random_position()

        # Publica o novo local aleatorio
        pub_alvo.publish(local_aleatorio)
        
        rospy.loginfo("Alvo atualizado para x: %f, y: %f", local_aleatorio.x, local_aleatorio.y)

        # Aguarda ate o proximo ciclo - frequencia 0.1Hz
        rate.sleep()   