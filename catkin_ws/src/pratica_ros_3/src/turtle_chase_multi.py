#!/usr/bin/env python
"""
turtle_chase é um programa para aprender a usar o ROS python.
Este programa controla várias tartarugas que devem perseguir uma outra tartaruga (Alvo)
na turtlesim arena.
Autor: Luiz C M Oliveira
"""
import rospy
from geometry_msgs.msg import Twist, Pose2D
from math import sqrt, pow, atan2, pi
from turtlesim.msg import Pose
from pratica_ros_3.msg import Alvo

# Variáveis para tartarugas
cmdVel = []
current = []
desired = []
pub_velocity = []
distance_errors = []

# Coeficientes gerais (para tartarugas que não são alvo)
K_x = 0.25
K_a = 0.25

# Coeficientes específicos para a tartaruga alvo (Goal_turtle)
K_x_goal = 1.0  # Velocidade maior para o alvo
K_a_goal = 1.0  # Ângulo maior para o alvo

# Tolerâncias
distanceTolerance = .3
angleTolerance = .2

Goal_turtle_index = 0  # Índice da tartaruga alvo inicial
Goal_turtle = Pose2D() # Posição do alvo

# Callback para atualização da posição e índice do alvo
def update_goal_callback(currentvalue):
    global Goal_turtle_index, Goal_turtle

    # Atualiza o índice e posição do alvo
    Goal_turtle_index = currentvalue.TurtleGoal
    Goal_turtle.x = currentvalue.x
    Goal_turtle.y = currentvalue.y

# Callback da posição de cada tartaruga
def update_Pose(i, currentPose):
    current[i].x = currentPose.x
    current[i].y = currentPose.y
    current[i].theta = currentPose.theta

# Inicializar variáveis de velocidade e de erro de distância
def misc_setup():
    for i in range(N_turtles):
        cmdVel.append(Twist())
        current.append(Pose2D())
        desired.append(Pose2D())
        distance_errors.append(0.0)

# Calcula o erro de distância entre uma tartaruga e o alvo
def getDistanceError(i):
    return sqrt(pow((Goal_turtle.x - current[i].x), 2) + pow((Goal_turtle.y - current[i].y), 2))

# Calcula o erro angular para a tartaruga i em relação ao alvo
def getAngularError(i):
    delta_y = Goal_turtle.y - current[i].y
    delta_x = Goal_turtle.x - current[i].x
    theta_alvo = atan2(delta_y, delta_x)
    angular_error = theta_alvo - current[i].theta

    if angular_error > pi:
        angular_error -= 2 * pi
    elif angular_error < -pi:
        angular_error += 2 * pi

    return angular_error

# Define a velocidade e ângulo para cada tartaruga
def set_velocity_angle(i):
    global cmdVel, K_x, K_a, K_x_goal, K_a_goal, Goal_turtle_index
    
    # Verifica se a tartaruga atual não é a tartaruga alvo
    if i != Goal_turtle_index:
        # Define os coeficientes para as tartarugas não alvos
        K_x_current = K_x
        K_a_current = K_a
        
        # SE não for a tartaruga alvo
        # Atualiza a velocidade e o ângulo
        if getDistanceError(i) > distanceTolerance:
            if abs(getAngularError(i)) < angleTolerance:
                cmdVel[i].linear.x = K_x_current * getDistanceError(i)
                cmdVel[i].angular.z = 0
            else:
                cmdVel[i].angular.z = K_a_current * getAngularError(i)
                cmdVel[i].linear.x = 0
        else:
            # Para a tartaruga quando ela atinge o alvo (mas sem desativar o movimento permanentemente)
            cmdVel[i].linear.x = 0
            cmdVel[i].angular.z = 0
    # Caso contrário, se for a tartaruga alvo, não atualiza a velocidade
    return

### Rotina principal ###
if __name__ == "__main__":
    rospy.init_node('chase_turtle_multi_node')
    N_turtles = rospy.get_param('/turtle_numbers_param')
    Goal_turtle = Pose2D()

    misc_setup()

    # Subscrever aos tópicos de posição de cada tartaruga
    for i in range(N_turtles):
        rospy.Subscriber(f'/turtle{i}/pose', Pose, lambda data, idx=i: update_Pose(idx, data))
        pub_velocity.append(rospy.Publisher(f'/turtle{i}/cmd_vel', Twist, queue_size=10))

    # Subscrever ao tópico do alvo
    rospy.Subscriber("/chase_goal", Alvo, update_goal_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(N_turtles):
            set_velocity_angle(i)
            pub_velocity[i].publish(cmdVel[i])

            # Atualizar o erro de distância para cada tartaruga
            distance_errors[i] = getDistanceError(i)
            rospy.loginfo(f"Tartaruga {i} - Distância para o alvo: {distance_errors[i]:.2f}")

        rate.sleep()
