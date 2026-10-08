#This file is meant for plotting the results generated from C++ code 

import numpy as np
import matplotlib.pyplot as plt
import subprocess

subprocess.run(["../run_main.exe"],check="True")

#Needed parameters for plots
num_of_landmarks = 1e2
variance = 1e-1
radar_radius = 1e4
guess_radius_GN = 1e6
guess_radius_LM = 1e10



#First file is data for the L2-measure of the directional derivative
GNnoWGNdataUAV1_L2 = np.loadtxt("../output_data/L2Convergence_GN,NoWGN_UAV,1.csv",delimiter=",",skiprows=1)
GNnoWGNdataUAV1_iter = GNnoWGNdataUAV1_L2[:,0]
GNnoWGNdataUAV1_L2Error = GNnoWGNdataUAV1_L2[:,1]
GNnoWGNdataUAV2_L2 = np.loadtxt("../output_data/L2Convergence_GN,NoWGN_UAV,2.csv",delimiter=",",skiprows=1)
GNnoWGNdataUAV2_iter = GNnoWGNdataUAV2_L2[:,0]
GNnoWGNdataUAV2_L2Error = GNnoWGNdataUAV2_L2[:,1]

GNWGNdataUAV1_L2 = np.loadtxt("../output_data/L2Convergence_GN,WGN_UAV,1.csv",delimiter=",",skiprows=1)
GNWGNdataUAV1_iter = GNWGNdataUAV1_L2[:,0]
GNWGNdataUAV1_L2Error = GNWGNdataUAV1_L2[:,1]
GNWGNdataUAV2_L2 = np.loadtxt("../output_data/L2Convergence_GN,WGN_UAV,2.csv",delimiter=",",skiprows=1)
GNWGNdataUAV2_iter = GNWGNdataUAV2_L2[:,0]
GNWGNdataUAV2_L2Error = GNWGNdataUAV2_L2[:,1]

LMnoWGNdataUAV1_L2 = np.loadtxt("../output_data/L2Convergence_LM,NoWGN_UAV,1.csv",delimiter=",",skiprows=1)
LMnoWGNdataUAV1_iter = LMnoWGNdataUAV1_L2[:,0]
LMnoWGNdataUAV1_L2Error = LMnoWGNdataUAV1_L2[:,1]
LMnoWGNdataUAV2_L2 = np.loadtxt("../output_data/L2Convergence_LM,NoWGN_UAV,2.csv",delimiter=",",skiprows=1)
LMnoWGNdataUAV2_iter = LMnoWGNdataUAV2_L2[:,0]
LMnoWGNdataUAV2_L2Error = LMnoWGNdataUAV2_L2[:,1]


LMWGNdataUAV1_L2 = np.loadtxt("../output_data/L2Convergence_LM,WGN_UAV,1.csv",delimiter=",",skiprows=1)
LMWGNdataUAV1_iter = LMWGNdataUAV1_L2[:,0]
LMWGNdataUAV1_L2Error = LMWGNdataUAV1_L2[:,1]
LMWGNdataUAV2_L2 = np.loadtxt("../output_data/L2Convergence_LM,WGN_UAV,2.csv",delimiter=",",skiprows=1)
LMWGNdataUAV2_iter = LMWGNdataUAV2_L2[:,0]
LMWGNdataUAV2_L2Error = LMWGNdataUAV2_L2[:,1]

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Gauss-Newton, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}$: L2 Convergence solution", fontsize=12, fontweight='bold')
ax[0].plot(GNnoWGNdataUAV1_iter,GNnoWGNdataUAV1_L2Error)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
fig.suptitle(f"Gauss-Newton, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}$: L2 Convergence solution", fontsize=12, fontweight='bold')
ax[1].plot(GNnoWGNdataUAV2_iter,GNnoWGNdataUAV2_L2Error)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/GNnoWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Gauss-Newton, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}, \sigma = {variance}$: L2 Convergence solution", fontsize=12, fontweight='bold')

ax[0].plot(GNWGNdataUAV1_iter,GNWGNdataUAV1_L2Error)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
ax[1].plot(GNWGNdataUAV2_iter,GNWGNdataUAV2_L2Error)
fig.suptitle(f"Gauss-Newton, {num_of_landmarks} Landmarks,$r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}, \sigma = {variance}$: L2 Convergence solution", fontsize=12, fontweight='bold')

ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/GNWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks} Landmarks,$r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}$: L2 Convergence solution",
              fontsize=12, fontweight='bold')
ax[0].plot(LMnoWGNdataUAV1_iter,LMnoWGNdataUAV1_L2Error)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
fig.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks} Landmarks,$r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}$: L2 Convergence solution",
              fontsize=12, fontweight='bold')
ax[1].plot(LMnoWGNdataUAV2_iter,LMnoWGNdataUAV2_L2Error)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/LMnoWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Levenberg-Marquardt WGN, {num_of_landmarks} Landmarks,  $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}, \sigma = {variance}$: L2 Convergence solution", fontsize=12, fontweight='bold')

ax[0].plot(LMWGNdataUAV1_iter,LMWGNdataUAV1_L2Error)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
ax[1].plot(LMWGNdataUAV2_iter,LMWGNdataUAV2_L2Error)
fig.suptitle(f"Levenberg-Marquardt WGN, {num_of_landmarks} Landmarks,  $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}, \sigma = {variance}$: L2 Convergence solution", fontsize=12, fontweight='bold')

ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/LMWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

#The second file is data for the L2-measure of the residual distances relative to the landmarks
GNnoWGNdataUAV1_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_GN,NoWGN_UAV1.csv"
                                   ,delimiter=",",skiprows=1)
GNnoWGNdataUAV1_iterRes = GNnoWGNdataUAV1_L2Res[:,0]
GNnoWGNdataUAV1_L2ErrorRes = GNnoWGNdataUAV1_L2Res[:,1]
GNnoWGNdataUAV2_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_GN,NoWGN_UAV2.csv",delimiter=",",skiprows=1)
GNnoWGNdataUAV2_iterRes = GNnoWGNdataUAV2_L2Res[:,0]
GNnoWGNdataUAV2_L2ErrorRes = GNnoWGNdataUAV2_L2Res[:,1]

GNWGNdataUAV1_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_GN,WGN_UAV1.csv",delimiter=",",skiprows=1)
GNWGNdataUAV1_iterRes = GNWGNdataUAV1_L2Res[:,0]
GNWGNdataUAV1_L2ErrorRes = GNWGNdataUAV1_L2Res[:,1]
GNWGNdataUAV2_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_GN,WGN_UAV2.csv",delimiter=",",skiprows=1)
GNWGNdataUAV2_iterRes = GNWGNdataUAV2_L2Res[:,0]
GNWGNdataUAV2_L2ErrorRes = GNWGNdataUAV2_L2Res[:,1]

LMnoWGNdataUAV1_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_LM,NoWGN_UAV1.csv",delimiter=",",skiprows=1)
LMnoWGNdataUAV1_iterRes = LMnoWGNdataUAV1_L2Res[:,0]
LMnoWGNdataUAV1_L2ErrorRes = LMnoWGNdataUAV1_L2Res[:,1]
LMnoWGNdataUAV2_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_LM,NoWGN_UAV2.csv",delimiter=",",skiprows=1)
LMnoWGNdataUAV2_iterRes = LMnoWGNdataUAV2_L2Res[:,0]
LMnoWGNdataUAV2_L2ErrorRes = LMnoWGNdataUAV2_L2Res[:,1]


LMWGNdataUAV1_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_LM,WGN_UAV1.csv",delimiter=",",skiprows=1)
LMWGNdataUAV1_iterRes = LMWGNdataUAV1_L2Res[:,0]
LMWGNdataUAV1_L2ErrorRes = LMWGNdataUAV1_L2Res[:,1]
LMWGNdataUAV2_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_LM,WGN_UAV2.csv",delimiter=",",skiprows=1)
LMWGNdataUAV2_iterRes = LMWGNdataUAV2_L2Res[:,0]
LMWGNdataUAV2_L2ErrorRes = LMWGNdataUAV2_L2Res[:,1]

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Gauss-Newton, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}$: L2 Convergence residual", fontsize=12, fontweight='bold')
ax[0].plot(GNnoWGNdataUAV1_iterRes,GNnoWGNdataUAV1_L2ErrorRes)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
fig.suptitle(f"Gauss-Newton, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}$: L2 Convergence residual", fontsize=12, fontweight='bold')
ax[1].plot(GNnoWGNdataUAV2_iterRes,GNnoWGNdataUAV2_L2ErrorRes)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/GNnoWGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Gauss-Newton WGN, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}, \sigma = {variance}$: L2 Convergence residual",fontsize=12, fontweight='bold')
ax[0].plot(GNWGNdataUAV1_iterRes,GNWGNdataUAV1_L2ErrorRes)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
fig.suptitle(f"Gauss-Newton WGN, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}, \sigma = {variance}$: L2 Convergence residual",fontsize=12, fontweight='bold')
ax[1].plot(GNWGNdataUAV2_iterRes,GNWGNdataUAV2_L2ErrorRes)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/GN_WGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}$: L2 Convergence residual",
              fontsize=12, fontweight='bold')
ax[0].plot(LMnoWGNdataUAV1_iterRes,LMnoWGNdataUAV1_L2ErrorRes)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
fig.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}$: L2 Convergence residual",
              fontsize=12, fontweight='bold')
ax[1].plot(LMnoWGNdataUAV2_iterRes,LMnoWGNdataUAV2_L2ErrorRes)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/LMnoWGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Levenberg-Marquerdt WGN, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}, \sigma = {variance}$: L2 Convergence residual",fontsize=12, fontweight='bold')
ax[0].plot(LMWGNdataUAV1_iterRes,LMWGNdataUAV1_L2ErrorRes)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("L2 Norm")
fig.suptitle(f"Levenberg-Marquerdt, WGN, {num_of_landmarks} Landmarks,  $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}, \sigma = {variance}$: L2 Convergence residual",fontsize=12, fontweight='bold')
ax[1].plot(LMWGNdataUAV2_iterRes,LMWGNdataUAV2_L2ErrorRes)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/LM_WGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

#The third file is data representing elapsed time using numerical methods over states per sought UAV

GNnoWGNdataUAV1_Time = np.loadtxt("../output_data/ElapsedTime_GN,NoWGN_UAV1.csv"
                                   ,delimiter=",",skiprows=1)
GNnoWGNdataUAV1_iterTime = GNnoWGNdataUAV1_Time[:,0]
GNnoWGNdataUAV1_errorTime = GNnoWGNdataUAV1_Time[:,1]
GNnoWGNdataUAV2_Time = np.loadtxt("../output_data/ElapsedTime_GN,NoWGN_UAV2.csv",delimiter=",",skiprows=1)
GNnoWGNdataUAV2_iterTime = GNnoWGNdataUAV2_Time[:,0]
GNnoWGNdataUAV2_errorTime = GNnoWGNdataUAV2_Time[:,1]

GNWGNdataUAV1_Time = np.loadtxt("../output_data/ElapsedTime_GN,WGN_UAV1.csv",delimiter=",",skiprows=1)
GNWGNdataUAV1_iterTime = GNWGNdataUAV1_Time[:,0]
GNWGNdataUAV1_errorTime = GNWGNdataUAV1_Time[:,1]
GNWGNdataUAV2_Time = np.loadtxt("../output_data/ElapsedTime_GN,WGN_UAV2.csv",delimiter=",",skiprows=1)
GNWGNdataUAV2_iterTime = GNWGNdataUAV2_Time[:,0]
GNWGNdataUAV2_errorTime = GNWGNdataUAV2_Time[:,1]

LMnoWGNdataUAV1_Time = np.loadtxt("../output_data/ElapsedTime_LM,NoWGN_UAV1.csv",delimiter=",",skiprows=1)
LMnoWGNdataUAV1_iterTime = LMnoWGNdataUAV1_Time[:,0]
LMnoWGNdataUAV1_errorTime = LMnoWGNdataUAV1_Time[:,1]
LMnoWGNdataUAV2_Time = np.loadtxt("../output_data/ElapsedTime_LM,NoWGN_UAV2.csv",delimiter=",",skiprows=1)
LMnoWGNdataUAV2_iterTime = LMnoWGNdataUAV2_Time[:,0]
LMnoWGNdataUAV2_errorTime = LMnoWGNdataUAV2_Time[:,1]


LMWGNdataUAV1_Time = np.loadtxt("../output_data/ElapsedTime_LM,WGN_UAV1.csv",delimiter=",",skiprows=1)
LMWGNdataUAV1_iterTime = LMWGNdataUAV1_Time[:,0]
LMWGNdataUAV1_errorTime = LMWGNdataUAV1_Time[:,1]
LMWGNdataUAV2_Time = np.loadtxt("../output_data/ElapsedTime_LM,WGN_UAV2.csv",delimiter=",",skiprows=1)
LMWGNdataUAV2_iterTime = LMWGNdataUAV2_Time[:,0]
LMWGNdataUAV2_errorTime = LMWGNdataUAV2_Time[:,1]

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Gauss-Newton, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}$: Elapsed Time", fontsize=12, fontweight='bold')
ax[0].plot(GNnoWGNdataUAV1_iterTime,GNnoWGNdataUAV1_errorTime)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("Time")
fig.suptitle(f"Gauss-Newton, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}$: Elapsed Time", fontsize=12, fontweight='bold')
ax[1].plot(GNnoWGNdataUAV2_iterTime,GNnoWGNdataUAV2_errorTime)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/GNnoWGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Gauss-Newton WGN, {num_of_landmarks} Landmarks,  $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}, \sigma = {variance}$: Elapsed Time",fontsize=12, fontweight='bold')
ax[0].plot(GNWGNdataUAV1_iterTime,GNWGNdataUAV1_errorTime)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("Time")
fig.suptitle(f"Gauss-Newton WGN, {num_of_landmarks} Landmarks,  $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}, \sigma = {variance}$: Elapsed Time",fontsize=12, fontweight='bold')
ax[1].plot(GNWGNdataUAV2_iterTime,GNWGNdataUAV2_errorTime)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/GN_WGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks} Landmarks, ${radar_radius}, r_{{guess}} = {guess_radius_LM}$: Elapsed Time",fontsize=12, fontweight='bold')
ax[0].plot(LMnoWGNdataUAV1_iterTime,LMnoWGNdataUAV1_errorTime)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("Time")
fig.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks} Landmarks, ${radar_radius}, r_{{guess}} = {guess_radius_LM}$: Elapsed Time",fontsize=12, fontweight='bold')
ax[1].plot(LMnoWGNdataUAV2_iterTime,LMnoWGNdataUAV2_errorTime)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/LMnoWGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(1,2)
fig.suptitle(f"Levenberg-Marquerdt WGN, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}, \sigma = {variance}$: Elapsed Time",fontsize=12, fontweight='bold')
ax[0].plot(LMWGNdataUAV1_iterTime,LMWGNdataUAV1_errorTime)
ax[0].set_title("UAV1")
ax[0].set_xlabel("Iterations")
ax[0].set_ylabel("Time")
fig.suptitle(f"Levenberg-Marquerdt WGN, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}, \sigma = {variance}$: Elapsed Time",fontsize=12, fontweight='bold')
ax[1].plot(LMWGNdataUAV2_iterTime,LMWGNdataUAV2_errorTime)
ax[1].set_title("UAV2")
ax[1].set_xlabel("Iterations")
ax[1].set_ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/LM_WGN_ElapsedTime.png', dpi=300, bbox_inches='tight')

plt.show()

#Fourth file is data that represents the moving ground truth coordinates for each iteration
GNnoWGNdatatruth_uav1 = np.loadtxt("../output_data/MovingUAV_GN,noWGN1.csv",delimiter=",",skiprows=1)
GNnoWGNdataiter_uav1 = GNnoWGNdatatruth_uav1[:,0]
GNnoWGNxtruth_uav1 = GNnoWGNdatatruth_uav1[:,1]
GNnoWGNytruth_uav1 = GNnoWGNdatatruth_uav1[:,2]
GNnoWGNztruth_uav1 = GNnoWGNdatatruth_uav1[:,3]
GNnoWGNdatatruth_uav2 = np.loadtxt("../output_data/MovingUAV_GN,noWGN2.csv",delimiter=",",skiprows=1)
GNnoWGNdataiter_uav2 = GNnoWGNdatatruth_uav2[:,0]
GNnoWGNdataxtruth_uav2 = GNnoWGNdatatruth_uav2[:,1]
GNnoWGNdataytruth_uav2 = GNnoWGNdatatruth_uav2[:,2]
GNnoWGNdataztruth_uav2 = GNnoWGNdatatruth_uav2[:,3]

GNWGNdatatruth_uav1 = np.loadtxt("../output_data/MovingUAV_GN,WGN1.csv",delimiter=",",skiprows=1)
GNWGNdataiter_uav1 = GNWGNdatatruth_uav1[:,0]
GNWGNdataxtruth_uav1 = GNWGNdatatruth_uav1[:,1]
GNWGNdataytruth_uav1 = GNWGNdatatruth_uav1[:,2]
GNWGNdataztruth_uav1 = GNWGNdatatruth_uav1[:,3]
GNWGNdatatruth_uav2 = np.loadtxt("../output_data/MovingUAV_GN,WGN2.csv",delimiter=",",skiprows=1)
GNWGNdatatruthiter_uav2 = GNWGNdatatruth_uav2[:,0]
GNWGNxtruth_uav2 = GNWGNdatatruth_uav2[:,1]
GNWGNytruth_uav2 = GNWGNdatatruth_uav2[:,2]
GNWGNtztruth_uav2 = GNWGNdatatruth_uav2[:,3]

LMnoWGNdatatruth_uav1 = np.loadtxt("../output_data/MovingUAV_LM,noWGN1.csv",delimiter=",",skiprows=1)
LMnoWGNdataiter_uav1 = LMnoWGNdatatruth_uav1[:,0]
LMnoWGNxtruth_uav1 = LMnoWGNdatatruth_uav1[:,1]
LMnoWGNytruth_uav1 = LMnoWGNdatatruth_uav1[:,2]
LMnoWGNztruth_uav1 = LMnoWGNdatatruth_uav1[:,3]
LMnoWGNdatatruth_uav2 = np.loadtxt("../output_data/MovingUAV_LM,noWGN2.csv",delimiter=",",skiprows=1)
LMnoWGNdatatruthiter_uav2 = LMnoWGNdatatruth_uav2[:,0]
LMnoWGNdataxtruth_uav2 = LMnoWGNdatatruth_uav2[:,1]
LMnoWGNdataytruth_uav2 = LMnoWGNdatatruth_uav2[:,2]
LMnoWGNdataztruth_uav2 = LMnoWGNdatatruth_uav2[:,3]

LMWGNdatatruth_uav1 = np.loadtxt("../output_data/MovingUAV_LM,WGN1.csv",delimiter=",",skiprows=1)
LMWGNdataiter_uav1 = LMWGNdatatruth_uav1[:,0]
LMWGNdataxtruth_uav1 = LMWGNdatatruth_uav1[:,1]
LMWGNdataytruth_uav1 = LMWGNdatatruth_uav1[:,2]
LMWGNdataztruth_uav1 = LMWGNdatatruth_uav1[:,3]
LMWGNdatatruth_uav2 = np.loadtxt("../output_data/MovingUAV_LM,WGN2.csv",delimiter=",",skiprows=1)
LMWGNdatatruthiter_uav2 = LMWGNdatatruth_uav2[:,0]
LMWGNxtruth_uav2 = LMWGNdatatruth_uav2[:,1]
LMWGNytruth_uav2 = LMWGNdatatruth_uav2[:,2]
LMWGNtztruth_uav2 = LMWGNdatatruth_uav2[:,3]



#The fifth file is data that represents the estimated sensor coordinates for each iteration
GNnoWGNdataest_uav1 = np.loadtxt("../output_data/UAVConvergence_GN,noWGN_UAV1.csv",delimiter=",",skiprows=1)
GNnoWGNiterest_uav1 = GNnoWGNdataest_uav1[:,0]
GNnoWGNxest_uav1 = GNnoWGNdataest_uav1[:,1]
GNnoWGNyest_uav1 = GNnoWGNdataest_uav1[:,2]
GNnoWGNzest_uav1 = GNnoWGNdataest_uav1[:,3]
GNnoWGNdataest_uav2 = np.loadtxt("../output_data/UAVConvergence_GN,noWGN_UAV2.csv",delimiter=",",skiprows=1)
GNnoWGNiterest_uav2 = GNnoWGNdataest_uav2[:,0]
GNnoWGNxest_uav2 = GNnoWGNdataest_uav2[:,1]
GNnoWGNyest_uav2 = GNnoWGNdataest_uav2[:,2]
GNnoWGNzest_uav2 = GNnoWGNdataest_uav2[:,3]

GNWGNdataest_uav1 = np.loadtxt("../output_data/UAVConvergence_GN,WGN_UAV1.csv",delimiter=",",skiprows=1)
GNWGNiterest_uav1 = GNWGNdataest_uav1[:,0]
GNWGNxest_uav1 = GNWGNdataest_uav1[:,1]
GNWGNyest_uav1 = GNWGNdataest_uav1[:,2]
GNWGNzest_uav1 = GNWGNdataest_uav1[:,3]
GNWGNdataest_uav2 = np.loadtxt("../output_data/UAVConvergence_GN,WGN_UAV2.csv",delimiter=",",skiprows=1)
GNWGNiterest_uav2 = GNWGNdataest_uav2[:,0]
GNWGNxest_uav2 = GNWGNdataest_uav2[:,1]
GNWGNyest_uav2 = GNWGNdataest_uav2[:,2]
GNWGNzest_uav2 = GNWGNdataest_uav2[:,3]

LMnoWGNdataest_uav1 = np.loadtxt("../output_data/UAVConvergence_LM,noWGN_UAV1.csv",delimiter=",",skiprows=1)
LMnoWGNiterest_uav1 = LMnoWGNdataest_uav1[:,0]
LMnoWGNxest_uav1 = LMnoWGNdataest_uav1[:,1]
LMnoWGNyest_uav1 = LMnoWGNdataest_uav1[:,2]
LMnoWGNzest_uav1 = LMnoWGNdataest_uav1[:,3]
LMnoWGNdataest_uav2 = np.loadtxt("../output_data/UAVConvergence_LM,noWGN_UAV2.csv",delimiter=",",skiprows=1)
LMnoWGNiterest_uav2 = LMnoWGNdataest_uav2[:,0]
LMnoWGNxest_uav2 = LMnoWGNdataest_uav2[:,1]
LMnoWGNyest_uav2 = LMnoWGNdataest_uav2[:,2]
LMnoWGNzest_uav2 = LMnoWGNdataest_uav2[:,3]

LMWGNdataest_uav1 = np.loadtxt("../output_data/UAVConvergence_LM,WGN_UAV1.csv",delimiter=",",skiprows=1)
LMWGNiterest_uav1 = LMWGNdataest_uav1[:,0]
LMWGNxest_uav1 = LMWGNdataest_uav1[:,1]
LMWGNyest_uav1 = LMWGNdataest_uav1[:,2]
LMWGNzest_uav1 = LMWGNdataest_uav1[:,3]
LMWGNdataest_uav2 = np.loadtxt("../output_data/UAVConvergence_LM,WGN_UAV2.csv",delimiter=",",skiprows=1)
LMWGNiterest_uav2 = LMWGNdataest_uav2[:,0]
LMWGNxest_uav2 = LMWGNdataest_uav2[:,1]
LMWGNyest_uav2 = LMWGNdataest_uav2[:,2]
LMWGNzest_uav2 = LMWGNdataest_uav2[:,3]


fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking GN, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}$: UAV 1", fontsize=12, fontweight='bold')

#GN, no WGN

# Panel for x-axis
axs[0].plot(GNnoWGNiterest_uav1, GNnoWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[0].plot(GNnoWGNiterest_uav1, GNnoWGNxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs[0].axhline(y=GNnoWGNxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(GNnoWGNiterest_uav1, GNnoWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[1].plot(GNnoWGNiterest_uav1, GNnoWGNytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs[1].axhline(y=GNnoWGNytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(GNnoWGNiterest_uav1, GNnoWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[2].plot(GNnoWGNiterest_uav1, GNnoWGNztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs[2].axhline(y=GNnoWGNztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/GNTelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking GN, {num_of_landmarks} Landmarks, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_GN}$: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs[0].plot(GNnoWGNiterest_uav2, GNnoWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[0].plot(GNnoWGNiterest_uav2, GNnoWGNdataxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs[0].axhline(y=GNnoWGNdataxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(GNnoWGNiterest_uav2, GNnoWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[1].plot(GNnoWGNiterest_uav2, GNnoWGNdataytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs[1].axhline(y=GNnoWGNdataytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(GNnoWGNiterest_uav2, GNnoWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[2].plot(GNnoWGNiterest_uav2, GNnoWGNdataztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs[2].axhline(y=GNnoWGNdataztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/GNTelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()

# GN,with WGN

fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking GN WGN, {radar_radius}, $r_{{guess}} = {guess_radius_GN}, \sigma = {variance}$, {num_of_landmarks} Landmarks: UAV 1", fontsize=12, fontweight='bold')

# Panel for x-axis
axs[0].plot(GNWGNiterest_uav1, GNWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[0].plot(GNWGNiterest_uav1, GNWGNdataxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs[0].axhline(y=GNWGNdataxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(GNWGNiterest_uav1, GNWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[1].plot(GNWGNiterest_uav1, GNWGNdataytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs[1].axhline(y=GNWGNdataytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(GNWGNiterest_uav1, GNWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[2].plot(GNWGNiterest_uav1, GNWGNdataztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs[2].axhline(y=GNWGNdataztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/GN_WGN_TelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking GN WGN, ${radar_radius}, r_{{guess}} = {guess_radius_GN}, \sigma = {variance}$, {num_of_landmarks} Landmarks: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs[0].plot(GNWGNiterest_uav2, GNWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[0].plot(GNWGNiterest_uav2, GNWGNxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs[0].axhline(y=GNWGNxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(GNWGNiterest_uav2, GNWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[1].plot(GNWGNiterest_uav2, GNWGNytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs[1].axhline(y=GNWGNytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(GNWGNiterest_uav2, GNWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[2].plot(GNWGNiterest_uav2, GNWGNtztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs[2].axhline(y=GNWGNtztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/GN_WGN_TelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()

#LM, no WGN
fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking LM, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}$, {num_of_landmarks} Landmarks: UAV 1", fontsize=12, fontweight='bold')
# Panel for x-axis
axs[0].plot(LMnoWGNiterest_uav1, LMnoWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[0].plot(LMnoWGNiterest_uav1, LMnoWGNxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs[0].axhline(y=LMnoWGNxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(LMnoWGNiterest_uav1, LMnoWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[1].plot(LMnoWGNiterest_uav1, LMnoWGNytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs[1].axhline(y=LMnoWGNytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(LMnoWGNiterest_uav1, LMnoWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[2].plot(LMnoWGNiterest_uav1, LMnoWGNztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs[2].axhline(y=LMnoWGNztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/LMTelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking LM, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}$, {num_of_landmarks} Landmarks: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs[0].plot(LMnoWGNiterest_uav2, LMnoWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[0].plot(LMnoWGNdatatruthiter_uav2, LMnoWGNdataxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs[0].axhline(y=LMnoWGNdataxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(LMnoWGNiterest_uav2, LMnoWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[1].plot(LMnoWGNdatatruthiter_uav2, LMnoWGNdataytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs[1].axhline(y=LMnoWGNdataytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(LMnoWGNiterest_uav2, LMnoWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[2].plot(LMnoWGNdatatruthiter_uav2, LMnoWGNdataztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs[2].axhline(y=LMnoWGNdataztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/LMTelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()

# LM,with WGN

fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking LM WGN, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}, \sigma = {variance}$, {num_of_landmarks} Landmarks: UAV 1", fontsize=12, fontweight='bold')

# Panel for x-axis
axs[0].plot(LMWGNiterest_uav1, LMWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[0].plot(LMWGNiterest_uav1, LMWGNdataxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs[0].axhline(y=LMWGNdataxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(LMWGNiterest_uav1, LMWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[1].plot(LMWGNiterest_uav1, LMWGNdataytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs[1].axhline(y=LMWGNdataytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(LMWGNiterest_uav1, LMWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs[2].plot(LMWGNiterest_uav1, LMWGNdataztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs[2].axhline(y=LMWGNdataztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/LM_WGN_TelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.suptitle(f"UAV Tracking LM WGN, $r_{{radar}} = {radar_radius}, r_{{guess}} = {guess_radius_LM}, \sigma = {variance}$, {num_of_landmarks} Landmarks: UAV 1", fontsize=12, fontweight='bold')

# Panel for x-axis
axs[0].plot(LMWGNiterest_uav2, LMWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[0].plot(LMWGNiterest_uav2, LMWGNxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs[0].axhline(y=LMWGNxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[0].set_ylabel("X-axis")

# Panel for y-axis
axs[1].plot(LMWGNiterest_uav2, LMWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[1].plot(LMWGNiterest_uav2, LMWGNytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs[1].axhline(y=LMWGNytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[1].set_ylabel("Y-axis")

# Panel for z-axis
axs[2].plot(LMWGNiterest_uav2, LMWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs[2].plot(LMWGNiterest_uav2, LMWGNtztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs[2].axhline(y=LMWGNtztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs[2].set_ylabel("Z-axis")

for ax in axs:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/LM_WGN_TelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()







