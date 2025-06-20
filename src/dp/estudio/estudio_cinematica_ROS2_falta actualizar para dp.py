#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 16 21:01:32 2024

@author: pablo
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov  2 20:20:55 2024

@author: pablo
"""

# Docu del robot myCobot320-PI
# https://docs.elephantrobotics.com/docs/mycobot-320-pi-en-test/

from DHRobotGT import DHRobotGT
from roboticstoolbox import RevoluteDH
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import scipy as sc
import matplotlib.pyplot as plt
from graficarCurvas import graficarTrayectorias
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


#%% Conexion con el robot
#from pymycobot import MyCobotSocket

# Set the serial connection, serial port, baud rate
# PI version
#mc = MyCobotSocket("10.42.0.1",9000)

#%%
def moverRobot(q):
    muestras = 100 
    #indices = np.linspace(0,q.shape[0]-1,num=muestras,dtype=int)
    for i in range(0,q.shape[0]):
        joint_state.header.stamp = ros2node.get_clock().now().to_msg()  # Marca de tiempo
        joint_state.position = list(q[i,:])  # Asignar las posiciones de los joints
        joint_publisher.publish(joint_state)  # Publicar el mensaje        
        time.sleep(0.01)


#%% Definicion del robot
#******************************************************************************
class dp(DHRobotGT):
  def __init__(self,*args,**kwargs):
    # Definición de los enlaces usando parámetros DH
    
    eje1 = RevoluteDH(alpha=0,a=0.2,d=0,offset=0)
    eje2 = RevoluteDH(alpha=0,a=0.2,d=0,offset=0)
    
    # Crear la estructura del robot
    super().__init__(*args,[eje1, eje2], name='dp',gravity = np.array([0, 0, -9.8]),**kwargs)

  # Luego de trabajar un rato en esto propuse la siguiente función:
  def calc_conf(self,q):
      conf = np.sign(q[1])
      return conf

  def ikine_a(self,POSE,conf=1):
      
    # Adecuo las variables POSE 
    px,py,pz = POSE.t
    nx,ny,nz = POSE.R[:,0]
    sx,sy,sz = POSE.R[:,1]
    ax,ay,az = POSE.R[:,2]
    

    # Tomo los largos de eslabones de la tabla DH
    a1 = self.links[0].a
    a2 = self.links[1].a

    if (px**2+py**2)>(a1+a2)**2:
        print("Brazo fuera de rango\n")
        return [],-1    

    # Calculo q2
    c2 = (px**2+py**2-(a1**2+a2**2))/(2*a1*a2)
    s2 = conf * np.sqrt(1-c2**2)
    q2 = np.arctan2(s2,c2)

    # Calculo q1
    s1 = (a2*(py*c2-px*s2)+a1*py)/(px**2+py**2)
    c1 = (a2*(py *s2 + px*c2) + a1*px)/(px**2+py**2)
    q1 = np.arctan2(s1,c1)

    
    q = np.array([q1,q2]) - self.offset
    # Limito q entre -pi y pi
    q = (q + np.pi) % (2 * np.pi) - np.pi
    status=1
    return q,status


#%% Modelización de la cinemática
#******************************************************************************
# Crear una instancia del cobot
Ts = 10E-3
vmax= 150*(np.pi/180)*np.ones((6,))
tacc = 300/200/2 # La aceleración máxima del eje es de 200°/s² => si voy a máxima velocidad y quiero pasar a máxima velocidad pero negativa, necesito 2 tacc
cobot = dp(Ts=Ts,tacc=tacc,vmax=vmax)

qz = np.zeros((6,))

print(cobot)

# Se supone que esto corresponde a qz
POSE0=cobot.fkine(qz)
print(POSE0)

# Prueba del pcd-pci
for i in range(10):
    q_real=np.random.randn(2)
    conf_original = cobot.calc_conf(q_real).flatten()
    # print("******************************\nq real: ",q_real)
    # print("Configuración original: ",conf_original)
    A=cobot.fkine(q_real)
    q,status =cobot.ikine_a(A,conf_original)
    config = cobot.calc_conf(q).flatten()
    print(f"Config Alcanzada: {config}  |q-q_final|={np.linalg.norm(q_real-q)}")

#%% Preparo el nodo de ros2 para escribir el valor de las juntas
rclpy.init()
ros2node = Node('test_configs')
joint_publisher = ros2node.create_publisher(JointState, '/joint_states', 10)
joint_state = JointState()
joint_state.name = ['joint1', 'joint2']


#%% Configuraciones
#******************************************************************************
# Me ayudo con el pci numérico
# Tengo una intuición en que las 3 singularidades me particionan el espacio en:
#   hombro +/-: revisar si p1n tiene coordenadas positivas o negativas
#   codo +/-: revisar el valor q3
#   muñeca +/-: revisar la articulación q5 

np.set_printoptions(precision=2)

time.sleep(5)
#mc.send_angles([0,0,0,0,0,0], 20)

# Para una POSE dada evalúo las 8 formas de alcanzarla
q = np.random.randn(6)
# -> 4 configuraciones probadas 
#q = np.array([ 0.3,  -0.13, -0.88,  0.97,  1.46, -0.14])

# Para los slides de la clase
#q = np.array([ 0,  -0.5, -0.7,  1.3,  1.3, 0.5])

# 6 configuraciones
#q=np.array([ 0.6,  -0.63,  1.48,  0.81, -0.47,  0.5 ])

POSE = cobot.fkine(q)
print("Evaluando configuraciones para alcanzar la POSE")
print(POSE)
print(f"conf0={cobot.calc_conf(q)} --> q0={q}")
conf_alcanzadas=0
for i in range(8):  # Evaluo las 8 configuraciones
    binario = format(i, '03b')  # '03b' convierte el número a binario con al menos 3 dígitos
    configuracion_deseada= [1 if bit == '1' else -1 for bit in binario]
    q,status = cobot.ikine_a(POSE,configuracion_deseada)    
    if status==1:
        config = cobot.calc_conf(q)
        print(f"conf={configuracion_deseada} --> q_final={q}")
        #angulos = q*180/np.pi
        #mc.send_angles(angulos.tolist(), 20)        
        
        joint_state.header.stamp = ros2node.get_clock().now().to_msg()  # Marca de tiempo
        joint_state.position = list(q)  # Asignar las posiciones de los joints
    
        joint_publisher.publish(joint_state)  # Publicar el mensaje
                
        # # Construir el mensaje de ROS2 en formato adecuado
        # joints_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
        
        # # Construir el comando ros2 topic pub
        # command = ['ros2', 'topic', 'pub', '/joint_states', 'sensor_msgs/msg/JointState',
        #         f"\"header:\n  stamp:\n    sec: $(date +%s)\n    nanosec: 0\n  frame_id: ''\n"
        #         f"name: {joints_names}\n"
        #         f"position: {list(q)}\" -r 1 -1"
        #         ]
        # command_str = ' '.join(command)
        
        # # Mostrar el comando como string
        # print(command_str)
        # subprocess.run(command_str, shell=True)    
        time.sleep(0.5)
        conf_alcanzadas+=1        
    else:
        print(f"conf={configuracion_deseada} --> No alcanzable")#" {config} {q}")
        
print(f"Configuraciones alcanzadas: {conf_alcanzadas}")

#%% Singularidades
#******************************************************************************
# Si pensamos en la singularidad del hombro, nos damos cuenta de que cuando el origen de la terna 5 quede en la circunferencia que lo define el mecanismo quedará bloqueado
# Entonces proponemos una POSE que debiera ser singular, y para estar seguros que O5 queda donde queremos, orientamos el eslabón 6 igual que el 0. De esta forma O6 y O5 difieren solo en altura
# Viendo el gráfico el radio de la circunferencia prohibida es 88.78mm
POSE_sing=sm.SE3([[1,0,0,0],[0,1,0,-82],[0,0,1,450],[0,0,0,1]])
# Ahora busco q a partir de esa POSE. Dificultad: no tengo un PCI analítico, y los iterativos no llegan hasta de la singularidad
q,_ = cobot.ikine_a(POSE_sing,[1,1,-1]) 
print("POSE en la singularidad propuesta:\n",cobot.fkine(q))

J0 = cobot.jacob0(q)
# Como no está exactamente en la singularidad J no nos dice demasiado
# Calculamos los valores singulares entonces
U, S, Vt = np.linalg.svd(J0)

# Mostrar los valores singulares
print(f"Valor singular más grande: {S[0]}    más pequeño:{S[-1]}")
# Se ve que no es exactamente singular. Voy a recuperar los nucleos aproximados a partir de U y Vt
print(f"Espacio nulo de Js: {Vt[-1,:].flatten()}")
print(f"Espacio nulo de Js transpuesta: {U[:,-1].flatten()}")

moverRobot(q.reshape(1, -1))

#%% Proponemos una trayectoria en línea recta que pase cerca de la zona prohibida
# en la dirección del nucleo
POSE_near_sing = sm.SE3([[1,0,0,-82.1],[0,1,0,0],[0,0,1,450],[0,0,0,1]])
delta_y = np.linspace(-200, 200,500)
POSES_c = [sm.SE3(0, dy, 0) * POSE_near_sing  for dy in delta_y]
print("Primera POSE de la trayectoria\n",POSES_c[0]) 
print("Ultima POSE de la trayectoria\n",POSES_c[-1])
# Calcular la cinemática inversa para cada pose
q = np.empty((0,cobot.n))
q_ant  = np.random.rand(6) #np.array([-0.2400273,   0.76997512,  1.29503045, -2.06500557,  0.2400273,   1.57079633]) #np.random.randn(6)
for pose in POSES_c:
    q_ant,status = cobot.ikine_a(pose,[1,1,-1])  # Calcular la cinemática inversa
    if status==1:   
        q = np.vstack([q,q_ant])

# ik_solutions ahora contiene las soluciones de IK para cada pose
POSES = cobot.fkine(q)
conf = cobot.calc_conf(q)
# Manipulabilidad a lo largo de la trayectoria
W = np.vstack([cobot.manipulability(q) for q in q[:,]])
graficarTrayectorias(q,POSES,cobot.Ts,W=W,conf=conf)

q_mirror = np.concatenate((q, q[::-1, :]), axis=0)
moverRobot(q_mirror)



#%% Codo
# si q3=0 (brazo extendido)
# si q3=+/-pi no cuenta por los límites en q3
#q_sing_codo = np.random.rand(6)*cobot.qlim[1]
print("************** CODO")
q_sing_codo = np.array([0,np.pi/2,0,0,np.pi/2,0])
print(f"q singular: {q_sing_codo}")
print("POSE en la singularidad propuesta:")
print(cobot.fkine(q_sing_codo))
J0 = cobot.jacob0(q_sing_codo)
print(f"Determinante del Jacobiano en la singularidad: {np.linalg.det(J0)}")
print(f"Espacio nulo de Js: {sc.linalg.null_space(J0)}")
print(f"Espacio nulo de Js transpuesta: {sc.linalg.null_space(J0.T)}")

#%% Muñeca
# *****************************************************************************
# si q5=n*pi con n Entero
q_sing_mune = np.array([0,-np.pi/2,np.pi/3,0,0,0])
print("************** MUÑECA")
print(f"q singular: {q_sing_mune}")
print("POSE en la singularidad propuesta:")
print(cobot.fkine(q_sing_mune))
J0 = cobot.jacob0(q_sing_mune)
print(f"Determinante del Jacobiano en la singularidad: {np.linalg.det(J0)}")
print(f"Espacio nulo de Js: {sc.linalg.null_space(J0).flatten()}")
print(f"Espacio nulo de Js transpuesta: {sc.linalg.null_space(J0.T).flatten()}")

# Calculo una trayectoria en el espacio nulo
# Partimos de la singularidad, variando q(3) en forma descendente y recalculando el vector q de manera de quedarnos en el núcleo de J en cada paso
delta = 0.005
N_samples=400
q = q_sing_mune.reshape(1,-1)
for i in range(N_samples-1):
    J0 = cobot.jacob0(q[-1,:])
    J_null = sc.linalg.null_space(J0).flatten()
    J_null[np.abs(J_null) < 1E-9] = 0
    J_null=J_null/J_null[2]*-1    
    q = np.vstack([q, q[-1,:] + J_null*delta]) 

POSES = cobot.fkine(q)

print("POSE inicial\n",cobot.fkine(q[0,]))
print("POSE final\n",cobot.fkine(q[-1,]))

graficarTrayectorias(q,POSES,cobot.Ts)

q_mirror = np.concatenate((q, q[::-1, :]), axis=0)
moverRobot(q_mirror)

#%% Pensemos ahora en un movimiento en línea recta que pasa cerca de la singularidad
# Nuevamente nos movemos en la dirección Y0
q_near_sing = q_sing_mune + np.array([0,0,0,0,0.05,0])
POSE_near_sing = cobot.fkine(q_near_sing)
conf_near_sing = cobot.calc_conf(q_near_sing)
delta_y = np.linspace(10, -10,400)
POSES_c = [sm.SE3(0, dy, 0) * POSE_near_sing  for dy in delta_y]
print("Primera POSE de la trayectoria\n",POSES_c[0]) 
print("Ultima POSE de la trayectoria\n",POSES_c[-1])
# Calcular la cinemática inversa para cada pose
q = np.empty((0,cobot.n)) 
q_ant  = np.random.rand(6) #q_near_sing + np.array([-0.5,0,0.01,0,0,0])
for pose in POSES_c:
    q_ant,status = cobot.ikine_a(pose,conf_near_sing)
    if status!=1:
        continue
    #q_ant = cobot.ikine_GN(pose.A,q0=q_ant).q  # Calcular la cinemática inversa
    q = np.vstack([q,q_ant])
POSES = cobot.fkine(q)
conf = cobot.calc_conf(q)
# Manipulabilidad a lo largo de la trayectoria
W = np.vstack([cobot.manipulability(q) for q in q[:,]])
graficarTrayectorias(q,POSES,cobot.Ts,W=W,conf=conf)

q_mirror = np.concatenate((q, q[::-1, :]), axis=0)
moverRobot(q_mirror)


#%%
# Destruir el nodo y apagar el cliente ROS2
ros2node.destroy_node()
rclpy.shutdown()



