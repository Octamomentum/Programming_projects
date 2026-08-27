#This file is meant for plotting the results generated from C++ code 

import numpy as np
import matplotlib.pyplot as plt
import subprocess

subprocess.run(["../run_main.exe"],check="True")

num_of_landmarks = 1e4

#First file is data for the L2-measure of the direction vector x_L2
dataUAV1_L2 = np.loadtxt("../output_data/L2Convergence_GN_UAV1.csv",delimiter=",",skiprows=1)
dataUAV1_iter = dataUAV1_L2[:,0]
dataUAV1_L2Error = dataUAV1_L2[:,1]
dataUAV2_L2 = np.loadtxt("../output_data/L2Convergence_GN_UAV2.csv",delimiter=",",skiprows=1)
dataUAV2_iter = dataUAV2_L2[:,0]
dataUAV2_L2Error = dataUAV2_L2[:,1]

fig, ax = plt.subplots(1,2)
fig.suptitle(f"L2 Convergence solution, {num_of_landmarks} Landmarks", fontsize=12, fontweight='bold')
ax[0].plot(dataUAV1_iter,dataUAV1_L2Error)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
ax[1].plot(dataUAV2_iter,dataUAV2_L2Error)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

#The second file is data for the L2-measure of the residual distances relative to the landmarks
dataUAV1_RE = np.loadtxt("../output_data/ResidualErrorConvergence_GN_UAV1.csv",delimiter=",",skiprows=1)
dataUAV1_iterRE = dataUAV1_RE[:,0]
data_REError_UAV1 = dataUAV1_RE[:,1]
dataUAV2_RE = np.loadtxt("../output_data/ResidualErrorConvergence_GN_UAV2.csv",delimiter=",",skiprows=1)
dataUAV2_iterRE = dataUAV2_RE[:,0]
data_REError_UAV2 = dataUAV2_RE[:,1]

fig, ax = plt.subplots(1,2)
fig.suptitle(f"L2 Convergence Residual distances, {num_of_landmarks} Landmarks", fontsize=12, fontweight='bold')
ax[0].plot(dataUAV1_iter,data_REError_UAV1)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
ax[1].plot(dataUAV2_iter,data_REError_UAV2)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/L2ConvergenceResiduals.png', dpi=300, bbox_inches='tight')

plt.show()

#The third file is data representing elapsed time using G-N over iterations per sought UAV

dataUAV1_elapsedtime = np.loadtxt("../output_data/ElapsedTime_GN_UAV1.csv",delimiter=",",skiprows=1)
dataUAV1_itertime = dataUAV1_elapsedtime[:,0]
dataUAV1_time = dataUAV1_elapsedtime[:,1]
dataUAV2_elapsedtime = np.loadtxt("../output_data/ElapsedTime_GN_UAV2.csv",delimiter=",",skiprows=1)
dataUAV2_itertime = dataUAV2_elapsedtime[:,0]
dataUAV2_time = dataUAV2_elapsedtime[:,1]

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Elapsed time using G-N solver per iteration, {num_of_landmarks} Landmarks", fontsize=12, fontweight='bold')
ax[0].plot(dataUAV1_itertime,dataUAV1_time)
ax[0].set_title("UAV 1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("Time")
ax[1].plot(dataUAV2_itertime,dataUAV2_time)
ax[1].set_title("UAV 2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/ElapsedTimePerIteration.png', dpi=300, bbox_inches='tight')

plt.show()

#Fourth file is data that represents the moving ground truth coordinates for each iteration
datatruth_uav1 = np.loadtxt("../output_data/MovingUAV_1.csv",delimiter=",",skiprows=1)
iter_uav1 = datatruth_uav1[:,0]
xtruth_uav1 = datatruth_uav1[:,1]
ytruth_uav1 = datatruth_uav1[:,2]
ztruth_uav1 = datatruth_uav1[:,3]
datatruth_uav2 = np.loadtxt("../output_data/MovingUAV_2.csv",delimiter=",",skiprows=1)
iter_uav2 = datatruth_uav2[:,0]
xtruth_uav2 = datatruth_uav2[:,1]
ytruth_uav2 = datatruth_uav2[:,2]
ztruth_uav2 = datatruth_uav2[:,3]



#The fifth file is data that represents the estimated sensor coordinates for each iteration
dataest_uav1 = np.loadtxt("../output_data/UAVConvergence_GN_UAV1.csv",delimiter=",",skiprows=1)
iterest_uav1 = dataest_uav1[:,0]
xest_uav1 = dataest_uav1[:,1]
yest_uav1 = dataest_uav1[:,2]
zest_uav1 = dataest_uav1[:,3]
dataest_uav2 = np.loadtxt("../output_data/UAVConvergence_GN_UAV2.csv",delimiter=",",skiprows=1)
iterest_uav2 = dataest_uav2[:,0]
xest_uav2 = dataest_uav2[:,1]
yest_uav2 = dataest_uav2[:,2]
zest_uav2 = dataest_uav2[:,3]


fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking, UAV 1, {num_of_landmarks} Landmarks", fontsize=12, fontweight='bold')

# Panel for x-axis
axs[0].plot(iterest_uav1, xest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[0].plot(iter_uav1, xtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs[0].axhline(y=xtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(iterest_uav1, yest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[1].plot(iter_uav1, ytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs[1].axhline(y=ytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(iterest_uav1, zest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[2].plot(iter_uav1, ztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs[2].axhline(y=ztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/TelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking, UAV 2, {num_of_landmarks} Landmarks", fontsize=12, fontweight='bold')

# Panel for x-axis
axs[0].plot(iterest_uav2, xest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[0].plot(iter_uav2, xtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs[0].axhline(y=xtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(iterest_uav2, yest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[1].plot(iter_uav2, ytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs[1].axhline(y=ytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(iterest_uav2, zest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[2].plot(iter_uav2, ztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs[2].axhline(y=ztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")
plt.tight_layout()

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.savefig('../plots/TelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')

plt.show()







