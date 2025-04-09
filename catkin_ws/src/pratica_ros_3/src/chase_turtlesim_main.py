#!/usr/bin/env python

import random
import rospy
from pratica_ros_3.srv import Arena

"""
Programa para aprender o ROS
- Programa principal 
- Nó cliente que executa o serviço arena_mount
- Salva o numero de tartarugas no parametro /turtle_numbers_param
"""

def arena_mount_client():
    
    # Aguarda ate que o servico arena_mount esteja disponivel
    rospy.wait_for_service('arena_mount')
    try:
        # Cria um proxy para chamar o serviço 'arena_mount'
        arena_mount = rospy.ServiceProxy('arena_mount', Arena)
              
        # Chama o serviço para criação da arena
        response = arena_mount()

        # Verifica se a arena foi configurada com sucesso
        #if response.success:
        #    rospy.loginfo(f"Arena configurada com {num_turtles} tartarugas.")
        #else:
        #    rospy.logwarn("Falha ao configurar a arena.")
   
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    
    # Inicializa o node
    rospy.init_node('chase_turtles_node')

    # Chama o cliente para configurar a arena com o número de tartarugas
    arena_mount_client()
    
    #rospy.loginfo("Arena mount result is {result}")
