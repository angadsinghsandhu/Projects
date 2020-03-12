import numpy as np

#the localization problem can be seen as:

p = [0.2, 0.2, 0.2, 0.2, 0.2]
world = ['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
motion = [1, 1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):
    q = []
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i]*(hit*pHit + (1 - hit)*pMiss))
        # print(q)
    q = np.divide(q, sum(q))
    return q

def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact*p[(i - U)%len(p)]
        s = s + pOvershoot*p[(i - U - 1)%len(p)]
        s = s + pUndershoot*p[(i - U + 1)%len(p)]
        q.append(s)
    return q

# print(p)

for k in range(len(measurements)):
    p = sense(p, measurements[k])
    # print(p)
    p = move(p, motion[k])
    # print(p)


print(p)
