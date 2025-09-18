# GFOLD_parms_static

import numpy as np
from scipy import signal


def e_py3(i):
    return signal.unit_impulse(3, i)  # create a specific basis vector


def S_py3(_w_):  # _w_ to distinguish from our global namespace's w!
    return np.matrix([[0, -_w_[2], +_w_[1]],
                     [_w_[2], 0, -_w_[0]],
                     [-_w_[1], _w_[0], 0]])


'''------------------------ Hopper Example -------------------------- '''
# These are the numbers from the hopper

g0 = 9.80665         # standard gravity [m/s**2]
m_dry = 140         # dry mass kg
m_fuel = 60       # fuel in tons
T_max = 2000           # thrust max
throt = [0.1, 1]       # throttle ability
G_max = 10               # maximum allowable structural Gs
Isp = 203.94          # fuel efficiency (specific impulse)
V_max = 90              # velocity max
y_gs = np.radians(30)  # glide slope cone, must be 0 < Degrees < 90
p_cs = np.radians(45)  # thrust pointing constraint
alpha = 1/(Isp*g0)      # fuel consumption parameter
m_wet = (m_dry+m_fuel)  # wet mass kg
r1 = throt[0]*T_max  # lower thrust bound
r2 = throt[1]*T_max  # upper thrust bound

g = np.array([-3.71, 0, 0])                 # gravity
w = np.array([2.53*1e-5, 0, 6.62*1e-5])   # planetary angular velocity
# thrust vector reference direction
nh = np.array([1, 0, 0])

r_ = np.array([20, 5, 5])                 # initial position
v0 = np.array([0,  0,   0])                 # initial velocity

rf = np.array([0, 0, 0])                      # final position target
vf = np.array([0, 0, 0])                      # final velocity target

c = np.divide(e_py3(0), np.tan(y_gs))
E = np.array([[e_py3(0).T], [e_py3(1).T]])

A = np.empty([6, 6])
np.copyto(A[0:3, 0:3], np.zeros((3, 3)))  # top left
np.copyto(A[0:3, 3:6], np.eye(3))  # top right
np.copyto(A[3:6, 0:3], -np.square(S_py3(w)))  # bottom left
np.copyto(A[3:6, 3:6], np.multiply(-1, S_py3(w)))  # bottom right
B = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 0, 0],
             [0, 1, 0], [0, 0, 1]])  # 0vect and I

'''------------------------ Hopper Drone Example -------------------------- '''
# # These are the numbers from the hopper

# g0    = 9.80665         # standard gravity [m/s**2]
# m_dry = 1.1         # dry mass kg
# m_fuel= 0.1       # fuel in tons
# T_max = 17           # thrust max
# throt = [0.3,0.7]       # throttle ability
# G_max = 10               # maximum allowable structural Gs
# Isp   = 10000          # fuel efficiency (specific impulse)
# V_max = 90              # velocity max previous: 90
# y_gs  = np.radians(30)# glide slope cone, must be 0 < Degrees < 90
# p_cs  = np.radians(45)  # thrust pointing constraint
# alpha = 1/(Isp*g0)      # fuel consumption parameter
# m_wet = (m_dry+m_fuel)  # wet mass kg
# r1    = throt[0]*T_max  # lower thrust bound
# r2    = throt[1]*T_max  # upper thrust bound

# g = np.array([-3.71,0,0])                 # gravity
# w = np.array([2.53*1e-5, 0, 6.62*1e-5])   # planetary angular velocity
# nh= np.array([1,0,0])                     # thrust vector reference direction

# r_ = np.array([10, 0, 2])                 # initial position
# v0 = np.array([0,  0,   0])                 # initial velocity

# rf = np.array([0,0,0])                      # final position target
# vf = np.array([0,0,0])                      # final velocity target

# c = np.divide(e_py3(0),np.tan(y_gs))
# E = np.array( [ [e_py3(0).T],[e_py3(1).T] ] )

# A = np.empty([6,6])
# np.copyto(A[0:3,0:3] , np.zeros((3,3))     ) # top left
# np.copyto(A[0:3,3:6] , np.eye(3)           ) # top right
# np.copyto(A[3:6,0:3] , -np.square(S_py3(w))    ) # bottom left
# np.copyto(A[3:6,3:6] , np.multiply(-1,S_py3(w))) # bottom right
# B = np.array([[0,0,0],[0,0,0],[0,0,0],[1,0,0],[0,1,0],[0,0,1]]) # 0vect and I
