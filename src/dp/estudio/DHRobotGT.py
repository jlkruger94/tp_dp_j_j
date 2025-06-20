#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov  2 20:14:39 2024

@author: pablo
"""

# Extiendo la clase DHRobot para que incluya el generador de trayectorias joint y cartesiano
from roboticstoolbox import DHRobot
import spatialmath as sm
import numpy as np
#import matplotlib.pyplot as plt


class DHRobotGT(DHRobot):
  def __init__(self, *args, tacc=0.1, Ts=1E-3,vmax=np.array([2*np.pi,2*np.pi]), **kwargs):
    super().__init__(*args, **kwargs)
    self.tacc = tacc
    self.Ts = Ts
    self.vmax = vmax

  def interpoladorTrapezoidal(self,A,B,C,Tj):
    """
    Interpolador trapezoidal en zona 1 y 2

    Args:
      A: punto en el que estoy
      B: punto al que estaba yendo en el segmento anterior
      C: punto al que voy
      Tj: tiempo en que se realiza el movimiento

    Returns:
      q_aux: vector interpolado de posiciones
      qd_aux: vector interpolado de velocidades
      qdd_aux: vector interpolado de aceleraciones
    """
    DA = A-B
    DC = C-B

    # Zona 1
    # Generar el vector tseg para Zona 1
    tseg = np.arange(-self.tacc + self.Ts, self.tacc + self.Ts, self.Ts)

    # Calculo las referencias para zona 1
    qdd_aux = np.outer((DC/Tj+DA/self.tacc)/(2*self.tacc),np.ones(len(tseg)))
    qd_aux = (DC / Tj)[:, np.newaxis] * (tseg + self.tacc) / (2 * self.tacc) + (DA / self.tacc)[:, np.newaxis] * (tseg - self.tacc) / (2 * self.tacc)
    q_aux = (DC / Tj)[:, np.newaxis] * (tseg + self.tacc)**2 / (4 * self.tacc) + (DA / self.tacc)[:, np.newaxis] * (tseg - self.tacc)**2 / (4 * self.tacc) + np.outer(B,np.ones(len(tseg)))

    # Zona 2
    # Generar el vector tseg para Zona 2
    tseg = np.arange(self.tacc + self.Ts, Tj - self.tacc + 0.5* self.Ts , self.Ts)

    # Inicializar las matrices theta2p, thetap y theta
    qdd_aux = np.hstack([qdd_aux,np.zeros((len(B), len(tseg)))])   # Suponiendo que B es un vector
    qd_aux = np.hstack([qd_aux,np.outer(DC / Tj, np.ones(len(tseg)))])
    q_aux = np.hstack([q_aux,np.outer(DC / Tj, tseg) +  np.outer(B,np.ones(len(tseg)))])
    return q_aux,qd_aux,qdd_aux

  def genTrJoint(self, q_dest,Td):
    """
    Genera la trayectoria joint para un conjunto de puntos de paso

    Args:
      q_dest: Matriz con los puntos de paso. Cada fila corresponde a un punto
      Td: tiempos deseados de cada movimiento

    Returns:
      t: Vector de tiempo de referencia
      q: Vector de posiciones articulares de referencia
      qd: Vector de velocidades articulares de referencia
      qdd: Vector de aceleraciones articulares de referencia
      POSES: Vector de posiciones cartesianas de referencia
    """
    q = np.empty((self.nlinks,0)); qd = np.empty((self.nlinks,0)); qdd = np.empty((self.nlinks,0))
    A = q_dest[0,:];
    for i in range(len(q_dest)):
      B = q_dest[i,:]
      if i<len(q_dest)-1:
        C = q_dest[i+1,:]
      else:
        C = B
        Td[i] = 0
      Tj = np.max((np.max(np.abs(C-B)/self.vmax),Td[i],2*self.tacc))
      q_aux,qd_aux,qdd_aux = self.interpoladorTrapezoidal(A,B,C,Tj)
      q = np.hstack([q,q_aux]); qd = np.hstack([qd,qd_aux]); qdd = np.hstack([qdd,qdd_aux]);
      A = q[:,-1]
    t = np.linspace(0, q.shape[1],num=q.shape[1])*self.Ts

    # Calculo la trayectoria cartesiana deseada
    POSES = self.fkine(q.transpose()) # .extend([self.fkine(q[:,i]) for i in range(q.shape[1])])
    return t,q.transpose(),qd.transpose(),qdd.transpose(),POSES

  def genTrCart(self,POSE_dest,Td):
    """
    Genera la trayectoria cartesiana para un conjunto de puntos de paso

    Args:
      POSE_dest: Lista con las POSES de paso
      Td: tiempos deseados de cada movimiento

    Returns:
      t: Vector de tiempo de referencia
      q: Vector de posiciones articulares de referencia
      qd: Vector de velocidades articulares de referencia
      qdd: Vector de aceleraciones articulares de referencia
      POSES: Vector de posiciones cartesianas de referencia
    """

    POSEA = POSE_dest[0]
    POSES = []
    for i in range(len(POSE_dest)):
      POSEB = POSE_dest[i]
      if i<len(POSE_dest)-1:
        POSEC = POSE_dest[i+1]
      else:
        POSEC  = POSEB 
        Td[i] = 0
      A = np.concatenate((POSEA.t,POSEA.eulervec()))
      B = np.concatenate((POSEB.t,POSEB.eulervec()))
      C = np.concatenate((POSEC.t,POSEC.eulervec()))
      Tj = np.max([Td[i],2*self.tacc])

      pos,_,_ = self.interpoladorTrapezoidal(A,B,C,Tj)
      POSES.extend([sm.SE3(pos[0:3,j])*sm.SE3.EulerVec(pos[3:,i]) for j in range(pos.shape[1])])

      POSEA = POSES[-1]

    q = np.zeros((len(POSES),self.nlinks))
    for i in range(len(POSES)):
      q[i,:],_ = self.ikine_a(POSES[i])


    # Obtengo la velocidad articular derivando numéricamente
    qd = np.diff(q, axis=0) / self.Ts
    # Ajustar la longitud de qd para que coincida con q
    qd = np.vstack([qd, np.zeros(self.nlinks,)])

    # Obtengo la aceleración articular derivando numéricamente
    qdd = np.diff(qd, axis=0) / self.Ts
    # Ajustar la longitud de qdd para que coincida con qd
    qdd = np.vstack([qdd, np.zeros(self.nlinks,)])

    t = np.linspace(0, len(q),num=len(q))*self.Ts
    return t,q,qd,qdd,POSES
