import numpy as np
import matplotlib.pyplot as plt
import subprocess

subprocess.run(["test_drone.exe"],check="True")

data_L2 = np.loadtxt("L2Convergence_GN.csv",delimiter=",",skiprows=1)
data_time = data_L2[:,0]
data_L2Error_UAV1 = data_L2[:,1]
data_L2Error_UAV2 = data_L2[:,2]
data_L2Error_UAV3 = data_L2[:,3]
data_L2Error_UAV4 = data_L2[:,4]

fig, ax = plt.subplots(2,2)
ax[0,0].plot(data_time,data_L2Error_UAV1)
ax[0,0].set_title("L2 Convergence solution UAV1")
ax[0,0].set_xlabel("Time")
ax[0,0].set_ylabel("L2 Norm")
ax[0,1].plot(data_time,data_L2Error_UAV2)
ax[0,1].set_title("L2 Convergence solution UAV2")
ax[0,1].set_xlabel("Time")
ax[0,1].set_ylabel("L2 Norm")
ax[1,0].plot(data_time,data_L2Error_UAV3)
ax[1,0].set_title("L2 Convergence solution UAV3")
ax[1,0].set_xlabel("Time")
ax[1,0].set_ylabel("L2 Norm")
ax[1,1].plot(data_time,data_L2Error_UAV4)
ax[1,1].set_title("L2 Convergence solution UAV4")
ax[1,1].set_xlabel("Time")
ax[1,1].set_ylabel("L2 Norm")
plt.show()

data_RE = np.loadtxt("ResidualErrorConvergence_GN.csv",delimiter=",",skiprows=1)
data_REError_UAV1 = data_RE[:,1]
data_REError_UAV2 = data_RE[:,2]
data_REError_UAV3 = data_RE[:,3]
data_REError_UAV4 = data_RE[:,4]

fig, ax = plt.subplots(2,2)
ax[0,0].plot(data_time,data_REError_UAV1)
ax[0,0].set_title("L2 Convergence solution UAV1")
ax[0,0].set_xlabel("Time")
ax[0,0].set_ylabel("L2 Norm")
ax[0,1].plot(data_time,data_REError_UAV2)
ax[0,1].set_title("L2 Convergence solution UAV2")
ax[0,1].set_xlabel("Time")
ax[0,1].set_ylabel("L2 Norm")
ax[1,0].plot(data_time,data_REError_UAV3)
ax[1,0].set_title("L2 Convergence solution UAV3")
ax[1,0].set_xlabel("Time")
ax[1,0].set_ylabel("L2 Norm")
ax[1,1].plot(data_time,data_REError_UAV4)
ax[1,1].set_title("L2 Convergence solution UAV4")
ax[1,1].set_xlabel("Time")
ax[1,1].set_ylabel("L2 Norm")
plt.show()

data_ref = np.loadtxt("UAVReferencepoints.csv",delimiter=",",skiprows=1)
x_refUAV1 = data_ref[0::3]
y_refUAV1 = data_ref[1::3]
z_refUAV1 = data_ref[2::3]
x_refUAV2 = data_ref[3::3]
y_refUAV2 = data_ref[4::3]
z_refUAV2 = data_ref[5::3]
x_refUAV3 = data_ref[6::3]
y_refUAV3 = data_ref[7::3]
z_refUAV3 = data_ref[8::3]
x_refUAV4 = data_ref[9::3]
y_refUAV4 = data_ref[10::3]
z_refUAV4 = data_ref[11::3]
data_lm = np.loadtxt("Landmarks_GN.csv",delimiter=",",skiprows=1)
x_lm = data_lm[:,0]
y_lm = data_lm[:,1]
z_lm = data_lm[:,2]
data_uav = np.loadtxt("UAVConvergence_GN.csv",delimiter=",",skiprows=1)
x_uav1 = data_uav[:,0]
y_uav1 = data_uav[:,1]
z_uav1 = data_uav[:,2]
x_uav2 = data_uav[:,3]
y_uav2 = data_uav[:,4]
z_uav2 = data_uav[:,5]
x_uav3 = data_uav[:,6]
y_uav3 = data_uav[:,7]
z_uav3 = data_uav[:,8]
x_uav4 = data_uav[:,9]
y_uav4 = data_uav[:,10]
z_uav4 = data_uav[:,11]

fig, ax = plt.subplots(2,2,subplot_kw={'projection':'3d'})
ax[0,0].scatter(x_refUAV1,y_refUAV1,z_refUAV1, label = 'Reference point for UAV 1', color = 'green', marker = 'o')
ax[0,0].scatter(x_lm,y_lm,z_lm, label = 'Landmarks', color = 'blue', marker = '^', s = 60, alpha = 0.2)
ax[0,0].plot(x_uav1,y_uav1,z_uav1, label = 'Converged path for UAV 1', color = 'black')
ax[0,0].scatter(x_uav1,y_uav1,z_uav1,color = 'red', marker = '.', s = 40, alpha = 0.6)
ax[0,0].set_title("Detection UAV 1")
ax[0,0].set_xlabel("X")
ax[0,0].set_ylabel("Y")
ax[0,0].set_zlabel("Z")
ax[0,0].legend()
ax[0,0].grid(True)
ax[0,1].scatter(x_refUAV2,y_refUAV2,z_refUAV2, label = 'Reference point for UAV 2', color = 'green', marker = 'o')
ax[0,1].scatter(x_lm,y_lm,z_lm, label = 'Landmarks', color = 'blue', marker = '^', s = 60, alpha = 0.2)
ax[0,1].plot(x_uav2,y_uav2,z_uav2, label = 'Converged path for UAV 2', color = 'black')
ax[0,1].scatter(x_uav2,y_uav2,z_uav2,color = 'red', marker = '.', s = 40, alpha = 0.6)
ax[0,1].set_title("Detection UAV 2")
ax[0,1].set_xlabel("X")
ax[0,1].set_ylabel("Y")
ax[0,1].set_zlabel("Z")
ax[0,1].legend()
ax[0,1].grid(True)
ax[1,0].scatter(x_refUAV3,y_refUAV3,z_refUAV3, label = 'Reference point for UAV 3', color = 'green', marker = 'o')
ax[1,0].scatter(x_lm,y_lm,z_lm, label = 'Landmarks', color = 'blue', marker = '^', s = 60, alpha = 0.2)
ax[1,0].plot(x_uav3,y_uav3,z_uav3, label = 'Converged path for UAV 3', color = 'black')
ax[1,0].scatter(x_uav3,y_uav3,z_uav3,color = 'red', marker = '.', s = 40, alpha = 0.6)
ax[1,0].set_title("Detection UAV 3")
ax[1,0].set_xlabel("X")
ax[1,0].set_ylabel("Y")
ax[1,0].set_zlabel("Z")
ax[1,0].legend()
ax[1,0].grid(True)
ax[1,1].scatter(x_refUAV4,y_refUAV4,z_refUAV4, label = 'Reference point for UAV 4', color = 'green', marker = 'o')
ax[1,1].scatter(x_lm,y_lm,z_lm, label = 'Landmarks', color = 'blue', marker = '^', s = 60, alpha = 0.2)
ax[1,1].plot(x_uav4,y_uav4,z_uav4, label = 'Converged path for UAV 4', color = 'black')
ax[1,1].scatter(x_uav4,y_uav4,z_uav4,color = 'red', marker = '.', s = 40, alpha = 0.6)
ax[1,1].set_title("Detection UAV 4")
ax[1,1].set_xlabel("X")
ax[1,1].set_ylabel("Y")
ax[1,1].set_zlabel("Z")
ax[1,1].legend()
ax[1,1].grid(True)
plt.show()







