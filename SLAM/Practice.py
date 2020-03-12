from math import *
from math import pi
import numpy as np

#standard gaussian equation

#mu is average, sigma^2 is variance, x is any point on the x axis
def f(mu, sigma2, x):
    return 1/((2.0*pi*sigma2)**(1/2))*exp(-0.5*(x-mu)**2/sigma2)

#belief of gaussian
print(f(10.,4.,8.))



#### 1-D Kalman Filter ####
#updating the 2 gaussians due to measurement
def update(mu1, var1, mu2, var2):
    new_mean =  (var1*mu2 + var2*mu1)/(var1 + var2)
    new_var = 1/(1/var1+1/var2)
    return [new_mean, new_var]

#predicting gaussian due to motion
def predict(mu1, var1, mu2, var2):
    new_mean =  mu1 + mu2
    new_var = var1 + var2
    return [new_mean, new_var]

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.#measurement uncertainty
motion_sig = 2. #motion uncertainty
mu = 0.
var = 10000.#uncertainty

for n in range(len(measurements)):
    [mu, var] = update(mu, var, measurements[n],measurement_sig)
    print('update :',mu, var)
    [mu, var] = predict(mu, var, motion[n], motion_sig)
    print('predict :',mu, var)

print (predict(10.,4.,12.,4.))

#x = estimate, P = uncertainty covariance/state covariance matrix, F = stste trasaction matrix,
#u = motion vector, z = Measurement,H = Measurement Function, R = measurement noise



#### 2-D kalman Filter ####
#Prediction Update
x_prime = np.dot(F,x.transpose()) + u #may also add B*u where 'B' is the control input and 'u' is control vector
#here x is a state transition function
P_prime = np.dot(np.dot(F,P),F.transpose())

#Measurement Update
y = z - H.dot(x) #error
S = H.dot(P).dot(H.transpose()) + R
K = np.dot(np.dot(P,H.transpose()),S.linalg.inv()) #kalman gain
x_prime = x +K*y
P_prime = (I - K*H)*P
