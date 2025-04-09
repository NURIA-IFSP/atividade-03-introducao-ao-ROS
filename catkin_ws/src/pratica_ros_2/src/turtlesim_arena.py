#!/usr/bin/env python

from math import pi
import random
import rospy
import subprocess
from turtlesim.srv import Spawn, SetPen


def start_turtlesim():
    # Inicia o nó ROS
    rospy.init_node('start_turtlesim_node', anonymous=True)

    # Executa o comando `rosrun turtlesim turtlesim_node` usando subprocess
    try:
        subprocess.Popen(["rosrun", "turtlesim", "turtlesim_node"])
        rospy.loginfo("Nó turtlesim_node iniciado com sucesso.")
    
    except Exception as e:
        rospy.logerr("Falha ao iniciar turtlesim_node: %s", e)


def spawn_turtle():
    # Espera o serviço estar disponível
    rospy.wait_for_service('/spawn')
    
    try:
        # Cria um objeto para acessar o serviço
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        

            # Gera um número aleatório entre 0 e 11
        x = random.uniform(0, 11)
        y = random.uniform(0, 11)
        theta = random.uniform(-pi, pi)

        # Chama o serviço para criar uma nova tartaruga
        # x, y são as coordenadas e theta é a orientação (ângulo)
        spawn_turtle(x, y, theta, "turtle2")  # x=5, y=5, theta=0, nome="turtle2"
        
        rospy.loginfo("Nova tartaruga criada com sucesso!")

    except rospy.ServiceException as e:
        rospy.logerr("Falha ao desativar a caneta 2: %s", e)


def set_pen_off():
    # Aguarda o serviço turtle1/set_pen estar disponível
    rospy.wait_for_service('/turtle2/set_pen')

    # Aguarda o serviço turtle1/set_pen estar disponível
    rospy.wait_for_service('/turtle1/set_pen')

    rospy.sleep(1)

    try:
        # Cria um proxy para o serviço /turtle1/set_pen
        set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)

        # Cria um proxy para o serviço /turtle2/set_pen
        set_pen2 = rospy.ServiceProxy('/turtle2/set_pen', SetPen)

        # Desativa a caneta configurando o parâmetro `off` para 1
        set_pen(0, 0, 0, 0, 1)
        
        # Desativa a caneta configurando o parâmetro `off` para 2
        set_pen2(0, 0, 0, 0, 1)
        
        rospy.loginfo("Caneta desativada para turtle1 e 2 .")

    except rospy.ServiceException as e:
        rospy.logerr("Falha ao desativar a caneta 1: %s", e)



if __name__ == "__main__":
    
    start_turtlesim()
    spawn_turtle()
    set_pen_off()
