#! /usr/bin/env python

"""
turtle_chase é um programa para aprender a usar o ROS python.
Este programa controla uma tartaruga 2 que deve perseguir a tartaruga 1
na turtlesim arena.

Autor: Luiz C M Oliveira
"""
## Import the Python library for ROS
import rospy

# Importa as mensagens do tipo Twist e Pose2D do pacote geometry_msgs
from geometry_msgs.msg import Twist, Pose2D

# funcoes matematicas
from math import sqrt, pow, atan2, pi

# importa a mensagem do tipo Pose do pacote turtlesim
from turtlesim.msg import Pose

# Declaração de variáveis

# Tartaruga 2
cmdVel2 = Twist()    # comando de velocidade
current2 = Pose2D()  # posicao atual
desired2 = Pose2D()  # posicao alvo

# Tartaruga 1
current1 = Pose2D()  # posicao atual
desired1 = Pose2D()  # posicao alvo


# Coeficiente (ou ganho) para o cálculo da velocidade linear
K_x = 0.75

# Coeficiente (ou ganho) para o cálculo do angulo derotacao em torno de z
K_a = 0.75

# Tolerancia aceita como "perto suficiente"
distanceTolerance = .3
angleTolerance = .1

# Flags e variáveis de controle
# wayponint sinaliza se a tartaruga está perto o suficiente do alvo
waypoint_active = True

# Callback da subscricao em turtle1/pose 
def update_Pose_turtle1(currentPose1):
  global current1
  current1.x = currentPose1.x
  current1.y = currentPose1.y
  current1.theta = currentPose1.theta
  # Apos obter a posicao de turtle1 Atualiza a posicao do alvo
  update_Alvo_turtle()
  # rospy.loginfo("Posicao turtle1: %f, y: %f", current1.x, current1.y)

# Callback da subscricao em /turtle2/pose
def update_Pose_turtle2(currentPose2):
  global current2
  current2.x = currentPose2.x
  current2.y = currentPose2.y
  current2.theta = currentPose2.theta  
  # rospy.loginfo("Posicao turtle2: %f, y: %f", current2.x, current2.y)
  
  

def update_Alvo_turtle():
   global desired2, waypoint_active
   desired2.x = current1.x
   desired2.y = current1.y
   waypoint_active = True  # Reativar o waypoint_active quando um novo alvo é definido
   # rospy.loginfo("Alvo atualizado para x: %f, y: %f", desired2.x, desired2.y)


# inicializacao das variaveis de velocidade de turtle2
def misc_setup():
  cmdVel2.linear.x = 0
  cmdVel2.linear.y = 0
  cmdVel2.linear.z = 0
  cmdVel2.angular.x = 0
  cmdVel2.angular.y = 0
  cmdVel2.angular.z = 0

# calculo do erro na distancia
def getDistanceError():
  return sqrt(pow((desired2.x - current2.x),2) + pow((desired2.y - current2.y),2))

# calculo do erro no angulo
def getAngularError():
   delta_y = desired2.y - current2.y
   delta_x = desired2.x - current2.x
   
   theta_alvo = atan2(delta_y,delta_x)
   angular_error = theta_alvo - current2.theta
   
   # Ajusta angular_error para o intervalo [-pi, pi]
   if angular_error > pi:
      angular_error = angular_error - 2*pi 
   elif angular_error < -pi:
      angular_error = angular_error + 2*pi
   
   return angular_error

# Se nao estiver perto suficiente ajuste o valor de cmd_vel
# Se estiver, ajuste para zero (para a tartaruga)
# 
def set_velocity_angle():
  global cmdVel2, waypoint_active

  # Se estiver no modo de busca (waypoint_acative=TRUE e longe do ponto alvo) 
  if waypoint_active == True and abs(getDistanceError()) > distanceTolerance:
     
     # Verifica se já está com theta alinhado ao alvo
     if abs(getAngularError()) < angleTolerance:
            # Se estiver, movimenta a tartaruga em relação ao ponto alvo
            cmdVel2.linear.x = K_x * getDistanceError() 
            cmdVel2.angular.z = 0
     else:
            # Se não estiver, rotaciona a tartaruga para alinhar ao ponto alvo
            cmdVel2.angular.z = K_a * getAngularError()
            cmdVel2.linear.x = 0
  else:
    # Se já chegou ao ponto alvo
    rospy.loginfo("I got you!")
    cmdVel2.linear.x = 0
    cmdVel2.angular.z = 0
    waypoint_active = False

    
# Exemplo de chamada de configuração
if __name__ == "__main__":
        
    # Inicializa o topico go_to_x - conecta ao roscore
    rospy.init_node('chase_turtle')

    # chama funcao para iniciar a velocidade
    misc_setup()

    # Registra o no como subscritor, chama a funcao callback
    # quando receber um dado com a posicao da tartaruga 2
    sub = rospy.Subscriber('/turtle2/pose', Pose, update_Pose_turtle2)

    # Registra o no como subscritor, chama a funcao callback
    # quando receber um dado com a posicao da tartaruga 1
    sub = rospy.Subscriber('/turtle1/pose', Pose, update_Pose_turtle1)

    # Atualiza a velocidade da tartaruga
    # Se vincula ao topico /turtle2/cmd_vel
    # para publicacao

    # Registrar o nó como um publicador
    pub_velocity = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    # Set a publish rate of 10 Hz
    rate = rospy.Rate(10)

    # Loop que sera executado ate o encerramento
    # da execucao do programa
    while not rospy.is_shutdown():
        # Ajusta a velocidade 
        set_velocity_angle()
        
        # Publica a velocidade - movimenta a tartaruga
        pub_velocity.publish(cmdVel2)
        
        # escreve na tela para acompanhamento
        """
        rospy.loginfo("goal x = %f", desired2.x)
        rospy.loginfo("goal y = %f", desired2.y)
        rospy.loginfo("current x = %f", current2.x)
        rospy.loginfo("current y = %f", current2.y)
        rospy.loginfo("disError = %f", getDistanceError())
        rospy.loginfo("angError = %f", getAngularError())
        rospy.loginfo("cmd_vel_x = %f", cmdVel2.linear.x)
        rospy.loginfo("cmd_vel_theta = %f", cmdVel2.angular.z)
        rospy.loginfo("waypoint_active = %s", waypoint_active)
        """        
        # Aguarda ate o proximo ciclo - frequencia 10Hz
        rate.sleep()                             