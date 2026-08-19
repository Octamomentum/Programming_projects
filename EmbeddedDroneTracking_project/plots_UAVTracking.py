#This file is meant for plotting the results generated from C++ code 

import numpy as np
import matplotlib.pyplot as plt
import subprocess

subprocess.run(["main_run.exe"],check="True")

#Initialize intended radius and centre for the radar
radar_radius = 1000
cx,cy,cz = 0,0,0

#Construct a unit sphere for amplifying the size for the radar system
sphere_phi = np.linspace(0,2*np.pi,100)
sphere_theta = np.linspace(0,np.pi,100)

x_unit = np.outer(np.cos(sphere_phi),np.sin(sphere_theta))
y_unit = np.outer(np.sin(sphere_phi),np.sin(sphere_theta))
z_unit = np.outer(np.ones(np.size(sphere_phi)),np.cos(sphere_theta))

x_inner,y_inner,z_inner = cx + radar_radius*x_unit, cy + radar_radius*y_unit, cz + radar_radius*z_unit 

#First file is data for the L2-measure of the direction vector x_L2

data_L2 = np.loadtxt("L2Convergence_GN.csv",delimiter=",",skiprows=1)
data_iter = data_L2[:,0]
data_L2Error_UAV1 = data_L2[:,1]
data_L2Error_UAV2 = data_L2[:,2]

fig, ax = plt.subplots(1,2)
ax[0].plot(data_iter,data_L2Error_UAV1)
ax[0].set_title("L2 Convergence solution UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
ax[1].plot(data_iter,data_L2Error_UAV2)
ax[1].set_title("L2 Convergence solution UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.show()

#The second file is data for the L2-measure of the distances between the landmarks
#and the estimated sensor coordinates
data_RE = np.loadtxt("ResidualErrorConvergence_GN.csv",delimiter=",",skiprows=1)
data_REError_UAV1 = data_RE[:,1]
data_REError_UAV2 = data_RE[:,2]

fig, ax = plt.subplots(1,2)
ax[0].plot(data_iter,data_REError_UAV1)
ax[0].set_title("L2 Convergence distances UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
ax[1].plot(data_iter,data_REError_UAV2)
ax[1].set_title("L2 Convergence distances UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")

plt.show()


#The third file is data for solving the least square over time per sought UAV
data_elapsedtime = np.loadtxt("ElapsedTimePerUAV_GN.csv",delimiter=",",skiprows=1)
data_itertime = data_elapsedtime[:,0]
data_timeUAV1 = data_elapsedtime[:,1]
data_timeUAV2 = data_elapsedtime[:,2]

fig, ax = plt.subplots(1,2)
ax[0].plot(data_itertime,data_timeUAV1)
ax[0].set_title("Time using G-N solver UAV1")
ax[0].set_xlabel("Elapsed time")
ax[0].set_ylabel("Time")
ax[1].plot(data_itertime,data_timeUAV2)
ax[1].set_title("Time using G-N solver UAV2")
ax[1].set_xlabel("Elapsed time")
ax[1].set_ylabel("Time")

plt.show()

#Fourth file is data that represents the coordinates for the landmarks
data_lm = np.loadtxt("Landmarks_GN.csv",delimiter=",",skiprows=1)
x_lm = data_lm[:,0]
y_lm = data_lm[:,1]
z_lm = data_lm[:,2]
#The fifth file is data that represents the estimated sensor coordinates for each time stamp
data_uav = np.loadtxt("UAVConvergence_GN.csv",delimiter=",",skiprows=1)
x_uav1 = data_uav[:,0]
y_uav1 = data_uav[:,1]
z_uav1 = data_uav[:,2]
x_uav2 = data_uav[:,3]
y_uav2 = data_uav[:,4]
z_uav2 = data_uav[:,5]

fig, ax = plt.subplots(1,2,subplot_kw={'projection':'3d'})
ax[0].plot_wireframe(x_inner,y_inner,z_inner,label='Radar',color='y',alpha=0.3,rstride=10,cstride=10) 
ax[0].scatter(x_lm,y_lm,z_lm, label = 'Landmarks', color = 'blue', marker = 'o', s = 10, alpha = 0.4)
ax[0].plot(x_uav1,y_uav1,z_uav1, label = 'Converged path for estimate UAV 1', color = 'black')
ax[0].scatter(x_uav1,y_uav1,z_uav1,color = 'red', marker = '.', s = 10, alpha=0.9)
ax[0].set_title("Detection UAV 1")
ax[0].set_xlabel("X")
ax[0].set_ylabel("Y")
ax[0].set_zlabel("Z")
ax[0].legend()
ax[0].grid(True)
ax[1].plot_wireframe(x_inner,y_inner,z_inner,label='Radar',color='y',alpha=0.2,rstride=10,cstride=10) 
ax[1].scatter(x_lm,y_lm,z_lm, label = 'Landmarks', color = 'blue', marker = 'o', s = 10, alpha = 0.4)
ax[1].plot(x_uav2,y_uav2,z_uav2, label = 'Converged path for estimate UAV 2', color = 'black')
ax[1].scatter(x_uav2,y_uav2,z_uav2,color = 'red', marker = '.', s = 10, alpha = 0.9)
ax[1].set_title("Detection UAV 2")
ax[1].set_xlabel("X")
ax[1].set_ylabel("Y")
ax[1].set_zlabel("Z")
ax[1].legend()
ax[1].grid(True)
plt.show()







