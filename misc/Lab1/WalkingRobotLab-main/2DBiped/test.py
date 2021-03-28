import numpy as np
import matplotlib.pyplot as plt

mat = np.zeros((50,6))
for i in range(int(5e3)):
    if True:
        t = i/500
        a = 0.3
        b = 0.15
        l = 0.6
        l1 = 0.5
        l2 = 0.5
        theta = (t*5-int(t))*(2*np.pi)
        y = a*np.cos(theta)
        x = b*np.sin(theta)+l
        cos_theta_2 = (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
        theta_2 = np.arctan2(np.sqrt(1-cos_theta_2**2), cos_theta_2)
        theta_1 = np.arctan2(y,x) + np.arctan2(l2*np.sin(theta_2),l1+l2*np.cos(theta_2))
        q_d_vec = np.array([theta_1, theta_2, np.pi/2, theta_1, theta_2, np.pi/2])
        phi_vec = np.r_[np.zeros(3), 0.1*np.ones(3)]
        q_d_vec = q_d_vec - phi_vec
        mat[:-1] = mat[1:]
        mat[-1] = q_d_vec
        plt.plot(mat[:,0])
        plt.pause(0.001)