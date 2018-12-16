from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

a = [x*0.02 for x in range(50)]

for i in range(len(a)): 
    b = [x*0.02 for x in range(50)]
    P_ab = [a[i]*b[j] for j in range(len(b))]
    P=[]
    for x in range(len(b)):
        P.append(a[i]+b[x]-P_ab[x])
    a_sub = [a[i] for y in range(len(b)) ] 
    ax.scatter(a_sub, b, P, c='r', marker='o')

ax.set_xlabel('P(A)')
ax.set_ylabel('P(B)')
ax.set_zlabel('P(A or B)')


plt.show()