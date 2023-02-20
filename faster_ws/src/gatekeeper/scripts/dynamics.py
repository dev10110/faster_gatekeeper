#!/usr/bin/env python

# Devansh Agrawal

# This file provides dynamics of a quadrotor


import math
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import Pose, Vector3, Quaternion
from gazebo_msgs.msg import ModelState
import numpy as np
from numpy import linalg as LA

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply, quaternion_matrix, quaternion_from_matrix

from visualization_msgs.msg import Marker

import tf

from utils import *



class Quadrotor:


    def __init__(self):

        self.mass = 1.0
        self.g = 9.81
        self.J = np.diag([0.03, 0.03, 0.06])
    
        self.kx_ = 9.5
        self.kv_ = 4.0
        self.kR_ = 3.35
        self.kw_ = 0.35

    def extract_state(self, np_state):
        x_np = np_state[0:3]
        v_np = np_state[3:6]
        R_np = np.reshape(np_state[6:15], [3,3])
        w_np = np.state[15:18]

        return x_np, v_np, R_np, w_np

    def extract_goal(np_goal):
        p = np_state[0:3]
        v = np_state[3:6]
        a = np_state[6:9]
        j = np_state[9:12]
        s = np_state[12:15]
        yaw   = np_state[15]
        dyaw  = np_state[16]
        ddyaw =  np_state[17]

        return p, v, a, j, s, yaw, dyaw, ddyaw


    def dynamics(self, state, goal, dt):
        # state is of type snapstack_msgs.msg.State: [pos, vel, quat, w, abias, gbias]
        # returns next state 

        # extract states
        x = vector3_to_np(state.pos)
        v = vector3_to_np(state.vel)
        R = quaternion_to_rotation(state.quat) 
        w = vector3_to_np(state.w)

        # convert goal to f, M 
        thrust, torque = self.goal_to_fM(state, goal)

        # define constants
        e3 = np.array([0,0,1.])

        # get dynamics
        xdot = v 
        vdot = -self.g * e3 + (thrust / self.mass) * np.matmul(R, e3)
        Rdot = np.matmul(R , hat(w))
        wdot = np.matmul(LA.inv(self.J), torque - np.cross(w, np.matmul(self.J, w)))

        # propagate (euler ugh)
        x_next = x + xdot * dt
        v_next = v + vdot * dt
        R_next = R + Rdot * dt
        w_next = w + wdot * dt

        # convert to new state
        state_next = State()
        state_next.pos   = np_to_vector3(x_next)
        state_next.vel   = np_to_vector3(v_next)
        state_next.quat  = np_rotation_to_quat(R_next)
        state_next.w     = np_to_vector3(w_next)
        state_next.abias = Vector3(0,0,0.)
        state_next.gbias = Vector3(0,0,0.)

        return state_next

    def geometric_controller(self, state, xd, vd, ad, b1d, wd, alphad):
        
        # extract state
        x = vector3_to_np(state.pos)
        v = vector3_to_np(state.vel)
        R = quaternion_to_rotation(state.quat)
        w = vector3_to_np(state.w)

        # controller gains
        kx = self.kx_ # 4.0
        kv = self.kv_ # 2.0
        kR = self.kR_ # 3.35
        kw = self.kw_ # 0.15
        
        # define constants
        e3 = np.array([0, 0, 1.0])

        # errors
        ex = x - xd
        ev = v - vd

        # construct desired rotation matrix
        fd =  -kx * ex - kv * ev + self.mass * self.g * e3 + self.mass * ad
        b3 = normalize(fd)
        b2 = normalize(np.cross(b3, normalize(b1d)))
        b1 = normalize(np.cross(b2, b3))

        Rd = np.column_stack([b1, b2, b3])

        eR = 0.5 * vee(np.matmul(Rd.T ,  R) - np.matmul(R.T , Rd) )
        ew = w - np.matmul(R.T, np.matmul(Rd, wd))

        f = np.dot(fd, np.matmul(R, e3))
        M =  -kR * eR - kw * ew + np.cross(w, np.matmul(self.J, w)) \
                - np.matmul(self.J ,  mmmm(hat(w),  R.T, Rd,  wd) - mmm(R.T, Rd , alphad) )

        return f, M


    def goal_to_fM(self, state, goal):
        
        # extract goal
        p = vector3_to_np(goal.p)
        v = vector3_to_np(goal.v)
        a = vector3_to_np(goal.a)
        j = vector3_to_np(goal.j)
        s = np.zeros(3) # since this is not specified in the goal
        
        yaw = goal.yaw
        dyaw = 0.0
        ddyaw = 0.0

        # desired rotation matrix
        force = np.array([a[0], a[1], a[2]+self.g])

        zb = normalize(force)

        xc = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        yb = normalize(np.cross(zb, xc))
        xb = normalize(np.cross(yb, zb))

        R = np.column_stack([xb, yb, zb])
        b1 = mm(R, np.array([1,0,0]))

        # construct tau
        tau = LA.norm(force)

        # construct S matrix
        bx1 = R[0, 0]
        bx2 = R[1, 0]
        bx3 = R[2, 0]
        by1 = R[0, 1]
        by2 = R[1, 1]
        by3 = R[2, 1]
        bz1 = R[0, 2]
        bz2 = R[1, 2]
        bz3 = R[2, 2]

        # 1x3 matrix
        S = np.array([
            0.0,
            (bx2 * bz1 - bx1 * bz2) / (bx1**2 + bx2**2),
            (-bx2 * by1 + bx1 * by2) / (bx1**2 + bx2**2)
        ])


        # solve for omega,  taudot
        iz = np.array([0, 0, 1])
        hatizT = hat(iz).T
        bz = R[:, 2]

        M = np.zeros([4,4]) 
        M[0:3, 0:3] = tau * mm(R , hatizT)
        M[0:3, 3] = bz
        M[3, 0:3] = S
        invM = LA.inv(M)

        tmp_v = np.zeros(4)
        tmp_v[0:3] = j
        tmp_v[3] = dyaw
        wtd = mm(invM , tmp_v)

        w = wtd[0:3]
        taud = wtd[3]

        # construct Sdot matrix
        # expression derived using mathematica
        w1 = w[0]
        w2 = w[1]
        w3 = w[2]

        Sd = np.array([
            0.0, 
            (bx1 * w1) / (bx1**2 + bx2**2) + (bx2 * w2) / (bx1**2 + bx2**2) + ((bx1**2 * bz1 - bx2**2 * bz1 + 2 * bx1 * bx2 * bz2) * w3) / (bx1**2 + bx2**2)**2, 
            ((bx1**2 * bx2 + bx2**3 - bx1**2 * by1 + bx2**2 * by1 - 2 * bx1 * bx2 * by2) * w3) / (bx1**2 + bx2**2)**2
        ])

        # solve for alpha, taudd
        B1 = mmm(R, 2 * taud * hatizT + tau * mm(hat(w), hatizT),  w)
        B2 = np.dot(Sd,  w)
        tmp_v2 = np.zeros(4)
        tmp_v2[0:3] = s - B1
        tmp_v2[3] = ddyaw - B2
        alpha_taudd = mm(invM, tmp_v2)

        alpha = alpha_taudd[0:3]
        # taudd = alpha_taudd[3]


        ## use geometric controller to convert to f, M
        f, M = self.geometric_controller(state, p, v, a, b1, w, alpha)

        return f, M



