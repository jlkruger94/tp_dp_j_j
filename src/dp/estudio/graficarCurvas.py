#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  3 15:39:06 2024

@author: pablo
"""

from scipy.spatial.transform import Rotation as Rotation
import matplotlib.pyplot as plt
import numpy as np

def calcularVelocidadAceleracion(q,Ts):
    qd = np.diff(q, axis=0) / Ts
    qd = np.vstack([qd, qd[-1,]])
    qdd = np.diff(qd, axis=0) / Ts
    qdd = np.vstack([qdd, qdd[-1,]])
    return qd,qdd
    
def graficarTrayectorias(q,POSES,Ts,conf=None,W=None):
    N_samples = q.shape[0]
    t = np.linspace(0,N_samples,N_samples)*Ts
    # Obtengo la velocidad joint derivando numéricamente
    qd,qdd = calcularVelocidadAceleracion(q,Ts)
        
    # Extraigo la posición del TCP para graficar
    pos = np.vstack(([pose.t[0] for pose in POSES], [pose.t[1] for pose in POSES],[pose.t[2] for pose in POSES])).T
    posd,posdd = calcularVelocidadAceleracion(pos,Ts)
    
    # Extraigo el vector de rotación
    alfa = np.vstack([Rotation.from_matrix(pose.R).as_rotvec() for pose in POSES])
    omega,gama = calcularVelocidadAceleracion(alfa,Ts)
    
    # Muestro las variables joint deseadas
    plt.subplot(3,1,1)
    labels = [rf'$q_{i+1}$' for i in range(q.shape[1])]
    for i in range(q.shape[1]):
        plt.plot(t, q[:,i]*180/np.pi, label=labels[i])
    plt.legend()
    plt.ylabel('q [°]')
    
    plt.title('Variables articulares')
    plt.subplot(3,1,2)
    plt.plot(t,qd*180/np.pi)
    plt.ylabel(r'$\dot{q}$ [°/s]')
    
    plt.subplot(3,1,3)
    plt.plot(t,qdd*180/np.pi)
    plt.ylabel(r'$\ddot{q}$ [°/$s^2$]')
    plt.show()
    
    # Muestro las variables cartesianas deseadas
    plt.subplot(3,1,1)
    plt.plot(t,pos)
    plt.legend(['x', 'y','z']);  plt.ylabel('Posición [mm]')
    plt.title('Variables cartesianas de posición')
    plt.subplot(3,1,2)
    plt.plot(t,posd)
    plt.legend([r'$v_x$', r'$v_y$',r'$v_z$']);  plt.ylabel('Vel [mm/s]')
    plt.subplot(3,1,3)
    plt.plot(t,posdd)
    plt.legend([r'$a_x$', r'$a_y$',r'$a_z$']); plt.xlabel('Tiempo [s]'); plt.ylabel(r'Acel $[mm/s^2]$')
    plt.show()

    # Muestro las variables cartesianas angulares deseadas
    plt.subplot(3,1,1)
    plt.plot(t,alfa*180/np.pi)
    plt.legend([r'$k_x$', r'$k_y$',r'$k_z$']);  plt.ylabel('Rotación [°]')
    plt.title('Variables cartesianas de rotación')
    plt.subplot(3,1,2)
    plt.plot(t,omega)
    plt.legend([r'$\omega_x$', r'$\omega_y$',r'$\omega_z$']);  plt.ylabel('Vel [°/s]')
    plt.subplot(3,1,3)
    plt.plot(t,gama)
    plt.legend([r'$\gamma_x$', r'$\gamma_y$',r'$\gamma_z$']); plt.xlabel('Tiempo [s]'); plt.ylabel(r'Acel [°/$s^2$]')
    plt.show()
    
    if W is not None:
        plt.figure()
        plt.plot(t,W)
        plt.xlabel('Tiempo [s]'); 
        plt.title('Manipulabilidad')
        plt.show()

    if conf is not None:
        plt.figure()
        plt.plot(t,conf)
        plt.xlabel('Tiempo [s]'); 
        plt.title('Configuración')
        plt.legend(['Hombro','Codo','Muñeca']);  plt.ylabel('Configuración')
        plt.show()
    
    # Muestro la trayectoria deseada
    # fig,ax = plt.subplots()
    # plt.plot(pos_ref[:,0],pos_ref[:,1])
    # circle = Circle((0, 0), alcanceXY,edgecolor='b', facecolor='none', linestyle='--')
    # ax.add_patch(circle)
    # plt.xlabel('x'); plt.ylabel('y')
    # plt.title(' Trayectoria de referencia')
    # plt.axis('equal')
    # plt.show()
