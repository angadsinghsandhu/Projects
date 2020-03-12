import numpy as np
import matplotlib.pyplot as plt
from math import *

class Matrix:

    #implementing a basic SLAM algorithm

    def __init__(self, value):
        #super(Filter, self).__init__()
        self.value = value
        self.dimx = dimx
        self.dimy = dimy
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        #check for correct dimentions
        if dimx < 0 or dimy < 0:
            raise ValueError("Invalid Size of Matrix")
            #raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        #check for correct dimentions
        if dim < 1:
            raise ValueError("Invalid Size of Matrix")
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 0

    def show(self):
        for i in range(dimx):
            print(self.value[i])
        print(' ')

    def __add__(self, other):
        #check for correct dimentions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Invalid Size of Matrix")
        else:
            res = matrix([[]])
            res.zero((self.dimx, self.dimy))
            for i in range(dimx):
                res.value[i] = self.value[i] + other.value[i]
            return res

    def __sub__(self, other):
        #check for correct dimentions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Invalid Size of Matrix")
        else:
            res = matrix([[]])
            res.zero((self.dimx, self.dimy))
            for i in range(dimx):
                res.value[i] = self.value[i] - other.value[i]
            return res

    def __mul__(self, other):
        #check for correct dimentions
        if self.dimy != other.dimx :
            raise ValueError("Invalid Size of Matrix")
        else:
            res = matrix([[]])
            res.zero((self.dimx, other.dimy))
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] = self.value[i][k]*other.value[k][j]
            return res

    def transpose(self):
        res = matrix([[]])
        res.zero((self.dimy, self.dimx))
        for i in range(dimx):
            for j in range(dimy):
                res.value[j][i] = self.value[i][j]

    def Chelosky(self, ztol = 1.0e-5):
        #compute upper triangular Cholosky factorization of a positive definitive Matrix
        res = matrix([[]])
        res.zero(self.dimx, self.dimy)

        for i in range(dimx):
            S = sum([(res.value[i][k])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol :
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError("Matrix is not Positive-Definite")
                res.value[i][i] = sqrt(d)
                for j in range():
                    for j in range(i+1, self.dimx):
                        S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                        if abs(S) < ztol:
                            S = 0.0
                            res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholosky()
        res = aux.CholoskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)

#####################################

#the Kalman Filter
def kalman_filter(x, P):
    for n in range(len(measurements)):
        #Measurement Update
        Z = np.matrix([[measurements[n]]])
        y = Z - (H*x)
        S = H*P*H.transpose() + R
        K = P*H.transpose()*np.linalg.inv(S)

        x = x - (K*y)
        P = (I - (K*H))*P

        #Prediction Step
        x = (F*x) + u
        P = F*P*F.transpose() #(2x4)(4x4)(4x2)

    return x, P

# Kalman Filter variables
# x; object state
# P; object covariance matrix
# S;
# K; Kalman Gain
# z; measurements
# u; external motion
# F; state transition matrix [F_h jacobian for EKF]
# H; measurement matrix [H_h jacobian for EKF]
# R; measurement covariance matrix
# I; Identity matrix
# Q; process covariance matrix
# x; Measurement Function [Range, Bearing, Range Rate]

############################################
### use the code below to test your filter!
############################################

measurements = [1, 2, 3]

x = np.matrix([[0.], [0.]]) # initial state (location and velocity)
P = np.matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = np.matrix([[0.], [0.]]) # external motion
F = np.matrix([[1., 1.], [0, 1.]]) # next state function
H = np.matrix([[1., 0.]]) # measurement function
R = np.matrix([[1.]]) # measurement uncertainty
I = np.matrix([[1., 0.], [0., 1.]]) # identity matrix

x_,P_ = kalman_filter(x, P)
print('x =',x_,'    \n','P = ',P_)
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]


class hello_abrar :
    def __init__(self,name):
        self.abrar = "hello_abrar"

s = hello_abrar("hey")
print(s.abrar)
