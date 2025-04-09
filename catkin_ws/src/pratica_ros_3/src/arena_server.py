#!/usr/bin/env python

import random
import rospy
import subprocess

from math import pi
from pratica_ros_3.srv import Arena, ArenaResponse
from turtlesim.srv import Spawn, SetPen


"""
Programa para aprendizado do ROS
- Disponibiliza o serviço ArenaMount que prepara a arena
- Não precisa do topico chase_goal este serviço mesmo gera o No de tartarugas e Alvo
- Salva o no. de tartarugas como um parametro do programa /turtle_number_param

"""

# Rotina executada quando o serviço é chamado
def handle_arena_mount(req):
    
    # Define o no de tartarugas
    new_turtles = random.randint(1, 5) # novas tartarugas - entre 1 e 5
    t_number = new_turtles + 1         # uma tartaruga já é criada com a abertura do turtlesim 
    rospy.loginfo("Turtles number is: %d", t_number)

    # Salva o parâmetro no servidor de parâmetros
    rospy.set_param('/turtle_numbers_param', t_number)

    
    # Cria a arena com o comando `rosrun turtlesim turtlesim_node` usando subprocess
    try:
        subprocess.Popen(["rosrun", "turtlesim", "turtlesim_node"])
        rospy.loginfo("Nó turtlesim_node iniciado com sucesso.")

        set_pen_off(1)  # desliga a caneta da turtle1

        for i in range(1, t_number):
            spawn_turtle(i+1)
            set_pen_off(i+1)
            rospy.loginfo("Turtle %d criada", i+1)
            rospy.sleep(0.2)

       
        result = True
    
    except Exception as e:
        rospy.logerr("Falha ao iniciar turtlesim_node: %s", e)
        result = False

    #rospy.loginfo("Service call result is: %s", result)
    rospy.loginfo("Arena pronta com %d tartarugas!", t_number)
    return ArenaResponse(result)


# Rotina que adiciona uma tartartura na arena
def spawn_turtle(turtle_number):
    # turtle_number - int - tartaruga que será criada
    
    turtle_name = f'/turtle{turtle_number}'

    # Espera o serviço estar disponível
    rospy.wait_for_service('/spawn')
    
    try:
        # Cria um objeto para acessar o serviço
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        
            # Gera um número aleatório entre 1 e 10 (para as tartarugas não ficarem escondidas)
        x = random.uniform(1, 10)
        y = random.uniform(1, 10)
        theta = random.uniform(-pi, pi)

        # Chama o serviço para criar uma nova tartaruga
        # x, y são as coordenadas e theta é a orientação (ângulo)
        spawn_turtle(x, y, theta, turtle_name)  
        
        rospy.loginfo("Tartaruga: %s criada com sucesso!", turtle_name)

    except rospy.ServiceException as e:
        rospy.logerr("Falha ao criar a tartaruga: %s", e)

# Rotina que desliga a linha de uma tartaruga
def set_pen_off(turtle_number):

    # Define o nome do serviço com base no número da tartaruga
    service_name = f'/turtle{turtle_number}/set_pen'
    turtle_name = f'/turtle{turtle_number}'

    # Aguarda o serviço turtleN/set_pen estar disponível
    rospy.wait_for_service(service_name)

    rospy.sleep(1)

    try:
        # Cria um proxy para o serviço /turtle1/set_pen
        set_pen = rospy.ServiceProxy(service_name, SetPen)   

        # Desativa a caneta configurando o parâmetro `off` para 1
        set_pen(0, 0, 0, 0, 1)
        
        rospy.loginfo("Caneta desativada para %s", turtle_name)

    except rospy.ServiceException as e:
        rospy.logerr("Falha ao desativar a caneta para: %s", turtle_name)


# Rotina que inicializa o serviço e conecta ao roscore
def arena_mount_server():
    rospy.init_node('arena_server_node')   

    service = rospy.Service('arena_mount', Arena, handle_arena_mount)
    rospy.loginfo("Service 'arena_mount' ready.")
    rospy.spin()


### Rotina principal - executada com o rosrun ###
if __name__ == "__main__":
    arena_mount_server()
