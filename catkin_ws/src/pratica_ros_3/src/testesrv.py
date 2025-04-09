#!/usr/bin/env python

import random
import rospy
from pratica_ros_3.srv import Arena

"""
Programa para aprender o ROS
- Nó cliente que executa o serviço arena_mount
"""

def arena_mount_client(num_turtles):
    
    # Aguarda ate que o servico arena_mount esteja disponivel
    rospy.wait_for_service('arena_mount')
    try:
        # Cria um proxy para chamar o serviço 'arena_mount'
        arena_mount = rospy.ServiceProxy('arena_mount', Arena)
              
        # Chama o serviço com o número de tartarugas especificado
        response = arena_mount(num_turtles)

        # Verifica se a arena foi configurada com sucesso
        if response.success:
            rospy.loginfo(f"Arena configurada com {num_turtles} tartarugas.")
        else:
            rospy.logwarn("Falha ao configurar a arena.")
   
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    
    # Inicializa o node
    rospy.init_node('arena_mount_client_node')
    
    # Define o no de tartarugas
    num_turtles = random.randint(1, 6)
    
    rospy.loginfo("Turtles number is: %d", num_turtles)

     # Define o parâmetro no servidor de parâmetros
    rospy.set_param('/turtle_numbers_param', num_turtles)

    # Chama o cliente para configurar a arena com o número de tartarugas
    arena_mount_client(num_turtles)
    
    #rospy.loginfo("Arena mount result is {result}")
