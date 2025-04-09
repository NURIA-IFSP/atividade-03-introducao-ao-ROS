#!/usr/bin/env python

from math import pi
from pratica_ros_3.msg import Alvo
import random
import rospy

"""
Nova versao:
Programa para aprendizado do ROS
- Utilização de parametros
- Cria um node "chase_goal_generator_node" que:
  - Obtem o numero de tartarugas do servidor de parametros 
  - Gera outro numero aleatorio entre 1 e no. tartarugas para determinar
    qual será o alvo
  - publica o topico /chase_goal com a mensagem Alvo.msg a cada 30s
"""

# Definicao de variaveis
Alvo_jogo = Alvo()  # Parametros do jogo - no. tartarugas e alvo

# Obtem o no. de tarturagas do servidor de parametros
Alvo_jogo.TurtleNum = rospy.get_param('/turtle_numbers_param')

# define qual será o alvo
# def set_turtle_goal(n_turtle):
#     Alvo_jogo.TurtleGoal = random.randint(1, n_turtle)
    # rospy.loginfo("Tartaruga Alvo: %d", Alvo_jogo.TurtleGoal)
    # Salva o parâmetro no servidor de parâmetros
#    rospy.set_param('/turtle_goal_param', Alvo_jogo.TurtleGoal)
#    return

# Define qual será o alvo
def set_turtle_goal(n_turtle):
    # Obtém o alvo anterior do servidor de parâmetros ou define como None caso não exista
    previous_goal = rospy.get_param('/turtle_goal_param', None)
    
    # Gera um novo alvo até que ele seja diferente do anterior
    new_goal = previous_goal
    while new_goal == previous_goal:
        new_goal = random.randint(1, n_turtle)
    
    # Atualiza o alvo e publica no servidor de parâmetros
    Alvo_jogo.TurtleGoal = new_goal
    rospy.set_param('/turtle_goal_param', Alvo_jogo.TurtleGoal)
    # rospy.loginfo("Novo Alvo: %d", Alvo_jogo.TurtleGoal)

    return



### Programa princiapal ###
if __name__ == "__main__":
        
    # Inicializa o node - conecta ao roscore
    rospy.init_node('chase_gen_node')

    # Registra o topico '/chase_goal' para publicacao de mensagens
    # do tipo Alvo
    pub = rospy.Publisher('/chase_goal', Alvo, queue_size=10)

    # Taxa de publicacao
    intervalo_de_tempo_entre_publicacoes = 10 #30s
    taxa_pub = float(1/intervalo_de_tempo_entre_publicacoes)

    rate = rospy.Rate(taxa_pub)

    # Loop que sera executado ate o encerramento
    # da execucao do programa
    while not rospy.is_shutdown():
        # Obtem o no de tartarugas do servidor de parametros
        n_turtle = rospy.get_param('/turtle_numbers_param')
        
        # Define qual será o alvo
        set_turtle_goal(n_turtle)

        # Publica a tartaruga alvo no jogo
        pub.publish(Alvo_jogo)
        
        # escreve na tela para acompanhamento
        
        # rospy.loginfo("No. de turtles = %d", Alvo_jogo.TurtleNum)
        rospy.loginfo("O alvo agora é a turtle %d", Alvo_jogo.TurtleGoal)
               
        # Aguarda ate o proximo ciclo - frequencia 1/no.publicacoes
        rate.sleep()         