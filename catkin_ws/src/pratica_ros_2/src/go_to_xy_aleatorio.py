#! /usr/bin/env python

# 
# go_to_xy.py eh programa para aprender a escrever ROS nodes.
# Este node comanda o turtlesim turtle para a posicao X e Y
# manualmente informada na constante do tipo double GOAL abaixo.
# Ao mudar o valor das constantes para qualquer valor entre 0 e 11 recarregue o programa
# Este programa eh uma conversao baseada no livro "Practical Robotics in C++"
# Escrito por Lloyd Brombach e publicado pela Packt Publishing
# Luiz Marangoni


# Import the Python library for ROS
import rospy

# Importa as mensagens do tipo Twist e Pose2D do pacote geometry_msgs
from geometry_msgs.msg import Twist, Pose2D

# funcoes matematicas
from math import sqrt, pow, atan2, pi

# importa a mensagem do tipo Pose do pacote turtlesim
from turtlesim.msg import Pose

# Declaração de variáveis
cmdVel = Twist()    # comando de velocidade
current = Pose2D()  # posicao atual
desired = Pose2D()  # posicao alvo

# Define o objetivo (pode ser qualquer valor de 0 a 11)
GOAL_X = 2
GOAL_Y = 10.5

# Coeficiente (ou ganho) para o cálculo da velocidade linear
K_x = 1.0

# Coeficiente (ou ganho) para o cálculo do angulo derotacao em torno de z
K_a = 1.0

# Tolerancia aceita como "perto suficiente"
distanceTolerance = .1
angleTolerance = .1

# Flags e variáveis de controle
# wayponint sinaliza se a tartaruga está perto o suficiente do alvo
waypoint_active = True

# Callback da subscricao em turtle1/pose chamada toda vez que
# um dado for recebido
# Obtem a posicao atual da tartaruga - currentPose e carrega
# na variavel current

def update_Pose(currentPose):
  global current
  current.x = currentPose.x
  current.y = currentPose.y
  current.theta = currentPose.theta

# Callback da subscrição em /alvo_aleatorio
def update_Alvo_aleatorio(currentAlvo):
   global desired, waypoint_active
   desired.x = currentAlvo.x
   desired.y = currentAlvo.y
   waypoint_active = True  # Reativar o waypoint_active quando um novo alvo é definido
   # rospy.loginfo("Alvo atualizado para x: %f, y: %f", desired.x, desired.y)


# inicializacao das variaveis de velocidade e pos desejada
def misc_setup():
  # desired.x e y obtidos a partir da entrada do usuario no topico /alvo
  # desired.x = GOAL_X
  # desired.y = GOAL_Y
  cmdVel.linear.x = 0
  cmdVel.linear.y = 0
  cmdVel.linear.z = 0
  cmdVel.angular.x = 0
  cmdVel.angular.y = 0
  cmdVel.angular.z = 0

# calculo do erro na distancia
def getDistanceError():
  return sqrt(pow((desired.x - current.x),2) + pow((desired.y - current.y),2))

# calculo do erro no angulo
def getAngularError():
   delta_y = desired.y - current.y
   delta_x = desired.x - current.x
   
   theta_alvo = atan2(delta_y,delta_x)
   angular_error = theta_alvo - current.theta
   
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
  global cmdVel, waypoint_active

  # Se estiver no modo de busca (waypoint_acative=TRUE e longe do ponto alvo) 
  if waypoint_active == True and abs(getDistanceError()) > distanceTolerance:
     
     # Verifica se já está com theta alinhado ao alvo
     if abs(getAngularError()) < angleTolerance:
            # Se estiver, movimenta a tartaruga em relação ao ponto alvo
            cmdVel.linear.x = K_x * getDistanceError() 
            cmdVel.angular.z = 0
     else:
            # Se não estiver, rotaciona a tartaruga para alinhar ao ponto alvo
            cmdVel.angular.z = K_a * getAngularError()
            cmdVel.linear.x = 0
  else:
    # Se já chegou ao ponto alvo
    rospy.loginfo("I'm here")
    cmdVel.linear.x = 0
    cmdVel.angular.z = 0
    waypoint_active = False

    
# Exemplo de chamada de configuração
if __name__ == "__main__":
        
    # Inicializa o topico go_to_x - conecta ao roscore
    rospy.init_node('go_to_xy_aleatorio')

    # chama funcao para iniciar a velocidade
    misc_setup()

    # Registra o no como subscritor, chama a funcao callback
    # quando receber um dado com a posicao da tartaruga
    sub = rospy.Subscriber('/turtle1/pose', Pose, update_Pose)

    # Registra o no como subscritor do no /alvo_aleatorio que gera
    # uma posicao aleatoria

    sub = rospy.Subscriber('/local_aleatorio', Pose2D, update_Alvo_aleatorio)

    # Atualiza a velocidade da tartaruga
    # Se vincula ao topico /turtle1/cmd_vel
    # para publicacao

    # Registrar o nó como um publicador
    pub_velocity = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Set a publish rate of 10 Hz
    rate = rospy.Rate(10)

    # Loop que sera executado ate o encerramento
    # da execucao do programa
    while not rospy.is_shutdown():
        # Ajusta a velocidade 
        set_velocity_angle()
        
        # Publica a velocidade - movimenta a tartaruga
        pub_velocity.publish(cmdVel)
        
        # escreve na tela para acompanhamento

        """ rospy.loginfo("goal x = %f", desired.x)
        rospy.loginfo("goal y = %f", desired.y)
        rospy.loginfo("current x = %f", current.x)
        rospy.loginfo("current y = %f", current.y)
        rospy.loginfo("disError = %f", getDistanceError())
        rospy.loginfo("angError = %f", getAngularError())
        rospy.loginfo("cmd_vel_x = %f", cmdVel.linear.x)
        rospy.loginfo("cmd_vel_theta = %f", cmdVel.angular.z)
        rospy.loginfo("waypoint_active = %s", waypoint_active)
        """
        # Aguarda ate o proximo ciclo - frequencia 10Hz
        rate.sleep()                             