#This file is meant for plotting the results generated from C++ code 

import numpy as np
import matplotlib.pyplot as plt
import subprocess

subprocess.run(["../run_main.exe"],check="True")

#Needed parameters for plots
num_of_landmarks = 1e3
variance = 1e-2
radar_radius = 1e3
guess_radius = 1e4

#First file is data for the L2-measure of the directional derivative
GNnoWGNdata_L2 = np.loadtxt("../output_data/L2Convergence_GN,NoWGN.csv",delimiter=",",skiprows=1)
GNnoWGNdata_iter = GNnoWGNdata_L2[:,0]
GNnoWGNdata_L2Error = GNnoWGNdata_L2[:,1]

GNWGNdata_L2 = np.loadtxt("../output_data/L2Convergence_GN,WGN.csv",delimiter=",",skiprows=1)
GNWGNdata_iter = GNWGNdata_L2[:,0]
GNWGNdata_L2Error = GNWGNdata_L2[:,1]


LMnoWGNdata_L2 = np.loadtxt("../output_data/L2Convergence_LM,NoWGN.csv",delimiter=",",skiprows=1)
LMnoWGNdata_iter = LMnoWGNdata_L2[:,0]
LMnoWGNdata_L2Error = LMnoWGNdata_L2[:,1]

LMWGNdata_L2 = np.loadtxt("../output_data/L2Convergence_LM,WGN.csv",delimiter=",",skiprows=1)
LMWGNdata_iter = LMWGNdata_L2[:,0]
LMWGNdata_L2Error = LMWGNdata_L2[:,1]

KFnoWGNdata_L2 = np.loadtxt("../output_data/L2Convergence_KALMANFILTER,NoWGN.csv",delimiter=",",skiprows=1)
KFnoWGNdata_iter = KFnoWGNdata_L2[:,0]
KFnoWGNdata_L2Error = KFnoWGNdata_L2[:,1]

KFWGNdata_L2 = np.loadtxt("../output_data/L2Convergence_KALMANFILTER,WGN.csv",delimiter=",",skiprows=1)
KFWGNdata_iter = LMWGNdata_L2[:,0]
KFWGNdata_L2Error = LMWGNdata_L2[:,1]



plt.suptitle(f"Gauss-Newton, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: L2 Convergence solution", fontsize=12, fontweight='bold')
plt.plot(GNnoWGNdata_iter,GNnoWGNdata_L2Error)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/GNnoWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Gauss-Newton WGN, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: L2 Convergence solution", fontsize=12, fontweight='bold')
plt.plot(GNWGNdata_iter,GNWGNdata_L2Error)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/GNWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks:g} Landmarks,$r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: L2 Convergence solution",
              fontsize=12, fontweight='bold')
plt.plot(LMnoWGNdata_iter,LMnoWGNdata_L2Error)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/LMnoWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Levenberg-Marquardt WGN, {num_of_landmarks:g} Landmarks,  $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: L2 Convergence solution", fontsize=12, fontweight='bold')

plt.plot(LMWGNdata_iter,LMWGNdata_L2Error)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/LMWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Kalman filter, {num_of_landmarks:g} Landmarks,$r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: L2 Convergence solution",
              fontsize=12, fontweight='bold')
plt.plot(KFnoWGNdata_iter,KFnoWGNdata_L2Error)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/KFnoWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Kalman filter WGN, {num_of_landmarks:g} Landmarks,  $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: L2 Convergence solution", fontsize=12, fontweight='bold')

plt.plot(KFWGNdata_iter,KFWGNdata_L2Error)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/KFWGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

#The second file is data for the L2-measure of the residual distances relative to the landmarks
GNnoWGNdata_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_GN,NoWGN.csv"
                                   ,delimiter=",",skiprows=1)
GNnoWGNdata_iterRes = GNnoWGNdata_L2Res[:,0]
GNnoWGNdata_L2ErrorRes = GNnoWGNdata_L2Res[:,1]

GNWGNdata_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_GN,WGN.csv",delimiter=",",skiprows=1)
GNWGNdata_iterRes = GNWGNdata_L2Res[:,0]
GNWGNdata_L2ErrorRes = GNWGNdata_L2Res[:,1]

LMnoWGNdata_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_LM,NoWGN.csv",delimiter=",",skiprows=1)
LMnoWGNdata_iterRes = LMnoWGNdata_L2Res[:,0]
LMnoWGNdata_L2ErrorRes = LMnoWGNdata_L2Res[:,1]

LMWGNdata_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_LM,WGN.csv",delimiter=",",skiprows=1)
LMWGNdata_iterRes = LMWGNdata_L2Res[:,0]
LMWGNdata_L2ErrorRes = LMWGNdata_L2Res[:,1]

KFnoWGNdata_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_KALMANFILTER,NoWGN.csv",delimiter=",",skiprows=1)
KFnoWGNdata_iterRes = KFnoWGNdata_L2Res[:,0]
KFnoWGNdata_L2ErrorRes = KFnoWGNdata_L2Res[:,1]

KFWGNdata_L2Res = np.loadtxt("../output_data/ResidualErrorConvergence_KALMANFILTER,WGN.csv",delimiter=",",skiprows=1)
KFWGNdata_iterRes = KFWGNdata_L2Res[:,0]
KFWGNdata_L2ErrorRes = KFWGNdata_L2Res[:,1]


plt.suptitle(f"Gauss-Newton, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: L2 Convergence residual", fontsize=12, fontweight='bold')
plt.plot(GNnoWGNdata_iterRes,GNnoWGNdata_L2ErrorRes)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/GNnoWGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Gauss-Newton WGN, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: L2 Convergence residual",fontsize=12, fontweight='bold')
plt.plot(GNWGNdata_iterRes,GNWGNdata_L2ErrorRes)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/GN_WGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: L2 Convergence residual",
              fontsize=12, fontweight='bold')
plt.plot(LMnoWGNdata_iter,LMnoWGNdata_L2ErrorRes)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/LMnoWGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Levenberg-Marquerdt WGN, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: L2 Convergence residual",fontsize=12, fontweight='bold')
plt.plot(LMWGNdata_iterRes,LMWGNdata_L2ErrorRes)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/LM_WGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Kalman filter, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: L2 Convergence residual",
              fontsize=12, fontweight='bold')
plt.plot(KFnoWGNdata_iter,KFnoWGNdata_L2ErrorRes)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/KFnoWGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Kalman filter WGN, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: L2 Convergence residual",fontsize=12, fontweight='bold')
plt.plot(KFWGNdata_iterRes,KFWGNdata_L2ErrorRes)
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.tight_layout()
plt.savefig('../plots/KF_WGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.show()

#The third file is data representing elapsed time using numerical methods over states

GNnoWGNdata_Time = np.loadtxt("../output_data/ElapsedTime_GN,NoWGN.csv"
                                   ,delimiter=",",skiprows=1)
GNnoWGNdata_iterTime = GNnoWGNdata_Time[:,0]
GNnoWGNdata_errorTime = GNnoWGNdata_Time[:,1]


GNWGNdata_Time = np.loadtxt("../output_data/ElapsedTime_GN,WGN.csv",delimiter=",",skiprows=1)
GNWGNdata_iterTime = GNWGNdata_Time[:,0]
GNWGNdata_errorTime = GNWGNdata_Time[:,1]


LMnoWGNdata_Time = np.loadtxt("../output_data/ElapsedTime_LM,NoWGN.csv",delimiter=",",skiprows=1)
LMnoWGNdata_iterTime = LMnoWGNdata_Time[:,0]
LMnoWGNdata_errorTime = LMnoWGNdata_Time[:,1]

LMWGNdata_Time = np.loadtxt("../output_data/ElapsedTime_LM,WGN.csv",delimiter=",",skiprows=1)
LMWGNdata_iterTime = LMWGNdata_Time[:,0]
LMWGNdata_errorTime = LMWGNdata_Time[:,1]

KFnoWGNdata_Time = np.loadtxt("../output_data/ElapsedTime_KALMANFILTER,NoWGN.csv",delimiter=",",skiprows=1)
KFnoWGNdata_iterTime = KFnoWGNdata_Time[:,0]
KFnoWGNdata_errorTime = KFnoWGNdata_Time[:,1]

KFWGNdata_Time = np.loadtxt("../output_data/ElapsedTime_KALMANFILTER,WGN.csv",delimiter=",",skiprows=1)
KFWGNdata_iterTime = KFWGNdata_Time[:,0]
KFWGNdata_errorTime = KFWGNdata_Time[:,1]


plt.suptitle(f"Gauss-Newton, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: Elapsed Time", fontsize=12, fontweight='bold')
plt.plot(GNnoWGNdata_iterTime,GNnoWGNdata_errorTime)
plt.xlabel("Iterations")
plt.ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/GNnoWGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Gauss-Newton WGN, {num_of_landmarks:g} Landmarks,  $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: Elapsed Time",fontsize=12, fontweight='bold')
plt.plot(GNWGNdata_iterTime,GNWGNdata_errorTime)
plt.xlabel("Iterations")
plt.ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/GN_WGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Levenberg-Marquerdt, {num_of_landmarks:g} Landmarks, ${radar_radius:g}, r_{{guess}} = {guess_radius:g}$: Elapsed Time",fontsize=12, fontweight='bold')
plt.plot(LMnoWGNdata_iterTime,LMnoWGNdata_errorTime)
plt.xlabel("Iterations")
plt.ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/LMnoWGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Levenberg-Marquerdt WGN, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: Elapsed Time",fontsize=12, fontweight='bold')
plt.plot(LMWGNdata_iterTime,LMWGNdata_errorTime)
plt.xlabel("Iterations")
plt.ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/LM_WGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Kalman filter, {num_of_landmarks:g} Landmarks, ${radar_radius:g}, r_{{guess}} = {guess_radius:g}$: Elapsed Time",fontsize=12, fontweight='bold')
plt.plot(KFnoWGNdata_iterTime,KFnoWGNdata_errorTime)
plt.xlabel("Iterations")
plt.ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/KFnoWGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(f"Kalman filter WGN, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$: Elapsed Time",fontsize=12, fontweight='bold')
plt.plot(KFWGNdata_iterTime,KFWGNdata_errorTime)
plt.xlabel("Iterations")
plt.ylabel("Time")
plt.tight_layout()
plt.savefig('../plots/KF_WGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.show()

#Fourth file is data that represents the moving ground truth coordinates for each iteration
GNnoWGNdatatruth_fleet = np.loadtxt("../output_data/MovingUAV_GN,noWGN.csv",delimiter=",",skiprows=1)
GNnoWGNdataiter_fleet = GNnoWGNdatatruth_fleet[:,0]
GNnoWGNxtruth_uav1 = GNnoWGNdatatruth_fleet[:,1]
GNnoWGNytruth_uav1 = GNnoWGNdatatruth_fleet[:,2]
GNnoWGNztruth_uav1 = GNnoWGNdatatruth_fleet[:,3]
GNnoWGNxtruth_uav2 = GNnoWGNdatatruth_fleet[:,4]
GNnoWGNytruth_uav2 = GNnoWGNdatatruth_fleet[:,5]
GNnoWGNztruth_uav2 = GNnoWGNdatatruth_fleet[:,6]

GNWGNdatatruth_fleet = np.loadtxt("../output_data/MovingUAV_GN,WGN.csv",delimiter=",",skiprows=1)
GNWGNdataiter_fleet = GNWGNdatatruth_fleet[:,0]
GNWGNxtruth_uav1 = GNWGNdatatruth_fleet[:,1]
GNWGNytruth_uav1 = GNWGNdatatruth_fleet[:,2]
GNWGNztruth_uav1 = GNWGNdatatruth_fleet[:,3]
GNWGNxtruth_uav2 = GNWGNdatatruth_fleet[:,4]
GNWGNytruth_uav2 = GNWGNdatatruth_fleet[:,5]
GNWGNztruth_uav2 = GNWGNdatatruth_fleet[:,6]

LMnoWGNdatatruth_fleet = np.loadtxt("../output_data/MovingUAV_LM,noWGN.csv",delimiter=",",skiprows=1)
LMnoWGNdataiter_fleet = LMnoWGNdatatruth_fleet[:,0]
LMnoWGNxtruth_uav1 = LMnoWGNdatatruth_fleet[:,1]
LMnoWGNytruth_uav1 = LMnoWGNdatatruth_fleet[:,2]
LMnoWGNztruth_uav1 = LMnoWGNdatatruth_fleet[:,3]
LMnoWGNxtruth_uav2 = LMnoWGNdatatruth_fleet[:,4]
LMnoWGNytruth_uav2 = LMnoWGNdatatruth_fleet[:,5]
LMnoWGNztruth_uav2 = LMnoWGNdatatruth_fleet[:,6]

LMWGNdatatruth_fleet = np.loadtxt("../output_data/MovingUAV_LM,WGN.csv",delimiter=",",skiprows=1)
LMWGNdataiter_fleet = LMWGNdatatruth_fleet[:,0]
LMWGNxtruth_uav1 = LMWGNdatatruth_fleet[:,1]
LMWGNytruth_uav1 = LMWGNdatatruth_fleet[:,2]
LMWGNztruth_uav1 = LMWGNdatatruth_fleet[:,3]
LMWGNxtruth_uav2 = LMWGNdatatruth_fleet[:,4]
LMWGNytruth_uav2 = LMWGNdatatruth_fleet[:,5]
LMWGNztruth_uav2 = LMWGNdatatruth_fleet[:,6]

KFnoWGNdatatruth_fleet = np.loadtxt("../output_data/MovingUAV_KALMANFILTER,noWGN.csv",delimiter=",",skiprows=1)
KFnoWGNdataiter_fleet = KFnoWGNdatatruth_fleet[:,0]
KFnoWGNxtruth_uav1 = KFnoWGNdatatruth_fleet[:,1]
KFnoWGNytruth_uav1 = KFnoWGNdatatruth_fleet[:,2]
KFnoWGNztruth_uav1 = KFnoWGNdatatruth_fleet[:,3]
KFnoWGNxtruth_uav2 = KFnoWGNdatatruth_fleet[:,4]
KFnoWGNytruth_uav2 = KFnoWGNdatatruth_fleet[:,5]
KFnoWGNztruth_uav2 = KFnoWGNdatatruth_fleet[:,6]

KFWGNdatatruth_fleet = np.loadtxt("../output_data/MovingUAV_KALMANFILTER,WGN.csv",delimiter=",",skiprows=1)
KFWGNdataiter_fleet = KFWGNdatatruth_fleet[:,0]
KFWGNxtruth_uav1 = KFWGNdatatruth_fleet[:,1]
KFWGNytruth_uav1 = KFWGNdatatruth_fleet[:,2]
KFWGNztruth_uav1 = KFWGNdatatruth_fleet[:,3]
KFWGNxtruth_uav2 = KFWGNdatatruth_fleet[:,4]
KFWGNytruth_uav2 = KFWGNdatatruth_fleet[:,5]
KFWGNztruth_uav2 = KFWGNdatatruth_fleet[:,6]

#The fifth file is data that represents the estimated sensor coordinates for each iteration
GNnoWGNdataest_fleet = np.loadtxt("../output_data/UAVConvergence_GN,noWGN.csv",delimiter=",",skiprows=1)
GNnoWGNdataiter_est_fleet = GNnoWGNdataest_fleet[:,0]
GNnoWGNxest_uav1 = GNnoWGNdataest_fleet[:,1]
GNnoWGNyest_uav1 = GNnoWGNdataest_fleet[:,2]
GNnoWGNzest_uav1 = GNnoWGNdataest_fleet[:,3]
GNnoWGNxest_uav2 = GNnoWGNdataest_fleet[:,4]
GNnoWGNyest_uav2 = GNnoWGNdataest_fleet[:,5]
GNnoWGNzest_uav2 = GNnoWGNdataest_fleet[:,6]

GNWGNdataest_fleet = np.loadtxt("../output_data/UAVConvergence_GN,WGN.csv",delimiter=",",skiprows=1)
GNWGNdataiter_est_fleet = GNWGNdataest_fleet[:,0]
GNWGNxest_uav1 = GNWGNdataest_fleet[:,1]
GNWGNyest_uav1 = GNWGNdataest_fleet[:,2]
GNWGNzest_uav1 = GNWGNdataest_fleet[:,3]
GNWGNxest_uav2 = GNWGNdataest_fleet[:,4]
GNWGNyest_uav2 = GNWGNdataest_fleet[:,5]
GNWGNzest_uav2 = GNWGNdataest_fleet[:,6]

LMnoWGNdataest_fleet = np.loadtxt("../output_data/UAVConvergence_LM,noWGN.csv",delimiter=",",skiprows=1)
LMnoWGNdataiter_est_fleet = LMnoWGNdataest_fleet[:,0]
LMnoWGNxest_uav1 = LMnoWGNdataest_fleet[:,1]
LMnoWGNyest_uav1 = LMnoWGNdataest_fleet[:,2]
LMnoWGNzest_uav1 = LMnoWGNdataest_fleet[:,3]
LMnoWGNxest_uav2 = LMnoWGNdataest_fleet[:,4]
LMnoWGNyest_uav2 = LMnoWGNdataest_fleet[:,5]
LMnoWGNzest_uav2 = LMnoWGNdataest_fleet[:,6]

LMWGNdataest_fleet = np.loadtxt("../output_data/UAVConvergence_LM,WGN.csv",delimiter=",",skiprows=1)
LMWGNdataiter_est_fleet = LMWGNdataest_fleet[:,0]
LMWGNxest_uav1 = LMWGNdataest_fleet[:,1]
LMWGNyest_uav1 = LMWGNdataest_fleet[:,2]
LMWGNzest_uav1 = LMWGNdataest_fleet[:,3]
LMWGNxest_uav2 = LMWGNdataest_fleet[:,4]
LMWGNyest_uav2 = LMWGNdataest_fleet[:,5]
LMWGNzest_uav2 = LMWGNdataest_fleet[:,6]

KFnoWGNdataest_fleet = np.loadtxt("../output_data/UAVConvergence_KALMANFILTER,noWGN.csv",delimiter=",",skiprows=1)
KFnoWGNdataiter_est_fleet = KFnoWGNdataest_fleet[:,0]
KFnoWGNxest_uav1 = KFnoWGNdataest_fleet[:,1]
KFnoWGNyest_uav1 = KFnoWGNdataest_fleet[:,2]
KFnoWGNzest_uav1 = KFnoWGNdataest_fleet[:,3]
KFnoWGNxest_uav2 = KFnoWGNdataest_fleet[:,4]
KFnoWGNyest_uav2 = KFnoWGNdataest_fleet[:,5]
KFnoWGNzest_uav2 = KFnoWGNdataest_fleet[:,6]

KFWGNdataest_fleet = np.loadtxt("../output_data/UAVConvergence_KALMANFILTER,WGN.csv",delimiter=",",skiprows=1)
KFWGNdataiter_est_fleet = KFWGNdataest_fleet[:,0]
KFWGNxest_uav1 = KFWGNdataest_fleet[:,1]
KFWGNyest_uav1 = KFWGNdataest_fleet[:,2]
KFWGNzest_uav1 = KFWGNdataest_fleet[:,3]
KFWGNxest_uav2 = KFWGNdataest_fleet[:,4]
KFWGNyest_uav2 = KFWGNdataest_fleet[:,5]
KFWGNzest_uav2 = KFWGNdataest_fleet[:,6]

#GN, no WGN

fig1, axs1 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig1.suptitle(f"UAV Tracking GN, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: UAV 1", fontsize=12, fontweight='bold')

# Panel for x-axis
axs1[0].plot(GNnoWGNdataiter_est_fleet, GNnoWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs1[0].plot(GNnoWGNdataiter_fleet, GNnoWGNxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs1[0].axhline(y=GNnoWGNxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs1[0].set_ylabel("X-axis")

# Panel for y-axis
axs1[1].plot(GNnoWGNdataiter_est_fleet, GNnoWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs1[1].plot(GNnoWGNdataiter_fleet, GNnoWGNytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs1[1].axhline(y=GNnoWGNytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs1[1].set_ylabel("Y-axis")

# Panel for z-axis
axs1[2].plot(GNnoWGNdataiter_est_fleet, GNnoWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs1[2].plot(GNnoWGNdataiter_fleet, GNnoWGNztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs1[2].axhline(y=GNnoWGNztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs1[2].set_ylabel("Z-axis")

for ax in axs1:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/GNTelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig2, axs2 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig2.suptitle(f"UAV Tracking GN, {num_of_landmarks:g} Landmarks, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs2[0].plot(GNWGNdataiter_est_fleet, GNWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs2[0].plot(GNWGNdataiter_fleet, GNWGNxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs2[0].axhline(y=GNWGNxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs2[0].set_ylabel("X-axis")

# Panel for y-axis
axs2[1].plot(GNWGNdataiter_est_fleet, GNWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs2[1].plot(GNWGNdataiter_fleet, GNWGNytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs2[1].axhline(y=GNWGNytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs2[1].set_ylabel("Y-axis")

# Panel for z-axis
axs2[2].plot(GNWGNdataiter_est_fleet, GNWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs2[2].plot(GNWGNdataiter_fleet, GNWGNztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs2[2].axhline(y=GNWGNztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs2[2].set_ylabel("Z-axis")

for ax in axs2:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/GNTelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()

# GN,with WGN

fig3, axs3 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig3.suptitle(f"UAV Tracking GN WGN, {radar_radius:g}, $r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$, {num_of_landmarks:g} Landmarks: UAV 1", fontsize=12, fontweight='bold')

# Panel for x-axis
axs3[0].plot(GNWGNdataiter_est_fleet, GNWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs3[0].plot(GNWGNdataiter_fleet, GNWGNxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs3[0].axhline(y=GNWGNxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs3[0].set_ylabel("X-axis")

# Panel for y-axis
axs3[1].plot(GNWGNdataiter_est_fleet, GNWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs3[1].plot(GNWGNdataiter_fleet, GNWGNytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs3[1].axhline(y=GNWGNytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs3[1].set_ylabel("Y-axis")

# Panel for z-axis
axs3[2].plot(GNWGNdataiter_est_fleet, GNWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs3[2].plot(GNWGNdataiter_fleet, GNWGNztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs3[2].axhline(y=GNWGNztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs3[2].set_ylabel("Z-axis")

for ax in axs3:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/GN_WGN_TelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig4, axs4 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig4.suptitle(f"UAV Tracking GN WGN, ${radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$, {num_of_landmarks:g} Landmarks: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs4[0].plot(GNWGNdataiter_est_fleet, GNWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs4[0].plot(GNWGNdataiter_fleet, GNWGNxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs4[0].axhline(y=GNWGNxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs4[0].set_ylabel("X-axis")

# Panel for y-axis
axs4[1].plot(GNWGNdataiter_est_fleet, GNWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs4[1].plot(GNWGNdataiter_fleet, GNWGNytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs4[1].axhline(y=GNWGNytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs4[1].set_ylabel("Y-axis")

# Panel for z-axis
axs4[2].plot(GNWGNdataiter_est_fleet, GNWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs4[2].plot(GNWGNdataiter_fleet, GNWGNztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs4[2].axhline(y=GNWGNztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs4[2].set_ylabel("Z-axis")

for ax in axs4:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/GN_WGN_TelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()

#LM, no WGN
fig5, axs5 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig5.suptitle(f"UAV Tracking LM, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$, {num_of_landmarks:g} Landmarks: UAV 1", fontsize=12, fontweight='bold')
# Panel for x-axis
axs5[0].plot(LMnoWGNdataiter_est_fleet, LMnoWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs5[0].plot(LMnoWGNdataiter_fleet, LMnoWGNxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs5[0].axhline(y=LMnoWGNxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs5[0].set_ylabel("X-axis")

# Panel for y-axis
axs5[1].plot(LMnoWGNdataiter_est_fleet, LMnoWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs5[1].plot(LMnoWGNdataiter_fleet, LMnoWGNytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs5[1].axhline(y=LMnoWGNytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs5[1].set_ylabel("Y-axis")

# Panel for z-axis
axs5[2].plot(LMnoWGNdataiter_est_fleet, LMnoWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs5[2].plot(LMnoWGNdataiter_fleet, LMnoWGNztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs5[2].axhline(y=LMnoWGNztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs5[2].set_ylabel("Z-axis")

for ax in axs5:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/LMTelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig6, axs6 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig6.suptitle(f"UAV Tracking LM, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$, {num_of_landmarks:g} Landmarks: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs6[0].plot(LMnoWGNdataiter_est_fleet, LMnoWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs6[0].plot(LMnoWGNdataiter_fleet, LMnoWGNxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs6[0].axhline(y=LMnoWGNxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs6[0].set_ylabel("X-axis")

# Panel for y-axis
axs6[1].plot(LMnoWGNdataiter_est_fleet, LMnoWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs6[1].plot(LMnoWGNdataiter_fleet, LMnoWGNytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs6[1].axhline(y=LMnoWGNytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs6[1].set_ylabel("Y-axis")

# Panel for z-axis
axs6[2].plot(LMnoWGNdataiter_est_fleet, LMnoWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs6[2].plot(LMnoWGNdataiter_fleet, LMnoWGNztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs6[2].axhline(y=LMnoWGNztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs6[2].set_ylabel("Z-axis")

for ax in axs6:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/LMTelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()

# LM,with WGN

fig7, axs7 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig7.suptitle(f"UAV Tracking LM WGN, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$, {num_of_landmarks:g} Landmarks: UAV 1", fontsize=12, fontweight='bold')

# Panel for x-axis
axs7[0].plot(LMWGNdataiter_est_fleet, LMWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs7[0].plot(LMWGNdataiter_fleet, LMWGNxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs7[0].axhline(y=LMWGNxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs7[0].set_ylabel("X-axis")

# Panel for y-axis
axs7[1].plot(LMWGNdataiter_est_fleet, LMWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs7[1].plot(LMWGNdataiter_fleet, LMWGNytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs7[1].axhline(y=LMWGNytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs7[1].set_ylabel("Y-axis")

# Panel for z-axis
axs7[2].plot(LMWGNdataiter_est_fleet, LMWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs7[2].plot(LMWGNdataiter_fleet, LMWGNztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs7[2].axhline(y=LMWGNztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs7[2].set_ylabel("Z-axis")

for ax in axs7:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/LM_WGN_TelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

#Kalmer filter, no WGN

fig8, axs8 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig8.suptitle(f"UAV Tracking LM WGN, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$, {num_of_landmarks:g} Landmarks: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs8[0].plot(LMWGNdataiter_est_fleet, LMWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs8[0].plot(LMWGNdataiter_fleet, LMWGNxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs8[0].axhline(y=LMWGNxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs8[0].set_ylabel("X-axis")

# Panel for y-axis
axs8[1].plot(LMWGNdataiter_est_fleet, LMWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs8[1].plot(LMWGNdataiter_est_fleet, LMWGNytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs8[1].axhline(y=LMWGNytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs8[1].set_ylabel("Y-axis")

# Panel for z-axis
axs8[2].plot(LMWGNdataiter_est_fleet, LMWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs8[2].plot(LMWGNdataiter_fleet, LMWGNztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs8[2].axhline(y=LMWGNztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs8[2].set_ylabel("Z-axis")

for ax in axs8:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/LM_WGN_TelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()

#LM, no WGN
fig9, axs9 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig9.suptitle(f"UAV Tracking Kalman filter, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$, {num_of_landmarks:g} Landmarks: UAV 1", fontsize=12, fontweight='bold')
# Panel for x-axis
axs9[0].plot(KFnoWGNdataiter_est_fleet, KFnoWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs9[0].plot(KFnoWGNdataiter_fleet, KFnoWGNxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs9[0].axhline(y=KFnoWGNxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs9[0].set_ylabel("X-axis")

# Panel for y-axis
axs9[1].plot(KFnoWGNdataiter_est_fleet, KFnoWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs9[1].plot(KFnoWGNdataiter_fleet, KFnoWGNytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs9[1].axhline(y=KFnoWGNytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs9[1].set_ylabel("Y-axis")

# Panel for z-axis
axs9[2].plot(KFnoWGNdataiter_est_fleet, KFnoWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs9[2].plot(KFnoWGNdataiter_fleet, KFnoWGNztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs9[2].axhline(y=KFnoWGNztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs9[2].set_ylabel("Z-axis")

for ax in axs9:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/KFTelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig10, axs10 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig10.suptitle(f"UAV Tracking Kalman filter, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}$, {num_of_landmarks:g} Landmarks: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs10[0].plot(KFnoWGNdataiter_est_fleet, KFnoWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs10[0].plot(KFnoWGNdataiter_fleet, KFnoWGNxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs10[0].axhline(y=KFnoWGNxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs10[0].set_ylabel("X-axis")

# Panel for y-axis
axs10[1].plot(KFnoWGNdataiter_est_fleet, KFnoWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs10[1].plot(KFnoWGNdataiter_fleet, KFnoWGNytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs10[1].axhline(y=KFnoWGNytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs10[1].set_ylabel("Y-axis")

# Panel for z-axis
axs10[2].plot(KFnoWGNdataiter_est_fleet, KFnoWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs10[2].plot(KFnoWGNdataiter_fleet, KFnoWGNztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs10[2].axhline(y=KFnoWGNztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs10[2].set_ylabel("Z-axis")

for ax in axs10:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/KFTelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()

# Kalmer filter,with WGN

fig11, axs11 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig11.suptitle(f"UAV Tracking Kalman filter WGN, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$, {num_of_landmarks:g} Landmarks: UAV 1", fontsize=12, fontweight='bold')

# Panel for x-axis
axs11[0].plot(KFWGNdataiter_est_fleet, KFWGNxest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs11[0].plot(KFWGNdataiter_fleet, KFWGNxtruth_uav1, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 1")
axs11[0].axhline(y=KFWGNxtruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs11[0].set_ylabel("X-axis")

# Panel for y-axis
axs11[1].plot(KFWGNdataiter_est_fleet, KFWGNyest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs11[1].plot(KFWGNdataiter_fleet, KFWGNytruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV1')
axs11[1].axhline(y=KFWGNytruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs11[1].set_ylabel("Y-axis")

# Panel for z-axis
axs11[2].plot(KFWGNdataiter_est_fleet, KFWGNzest_uav1, color='red', linewidth=2, label='Estimated coordinate, UAV 1')
axs11[2].plot(KFWGNdataiter_fleet, KFWGNztruth_uav1, color='blue', linewidth=2, label='Ground truth coordinate, UAV 1')
axs11[2].axhline(y=KFWGNztruth_uav1[-1], color='green', linestyle='--', alpha=0.5)
axs11[2].set_ylabel("Z-axis")

for ax in axs11:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/KF_WGN_TelemetryBenchmarkUAV1.png', dpi=300, bbox_inches='tight')
plt.show()

fig12, axs12 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig12.suptitle(f"UAV Tracking Kalman filter WGN, $r_{{radar}} = {radar_radius:g}, r_{{guess}} = {guess_radius:g}, \sigma = {variance:g}$, {num_of_landmarks:g} Landmarks: UAV 2", fontsize=12, fontweight='bold')

# Panel for x-axis
axs12[0].plot(KFWGNdataiter_est_fleet, KFWGNxest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs12[0].plot(KFWGNdataiter_fleet, KFWGNxtruth_uav2, color='blue', linewidth=2, label ="Ground truth coordinate, UAV 2")
axs12[0].axhline(y=KFWGNxtruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs12[0].set_ylabel("X-axis")

# Panel for y-axis
axs12[1].plot(KFWGNdataiter_est_fleet, KFWGNyest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs12[1].plot(KFWGNdataiter_fleet, KFWGNytruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV2')
axs12[1].axhline(y=KFWGNytruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs12[1].set_ylabel("Y-axis")

# Panel for z-axis
axs12[2].plot(KFWGNdataiter_est_fleet, KFWGNzest_uav2, color='red', linewidth=2, label='Estimated coordinate, UAV 2')
axs12[2].plot(KFWGNdataiter_fleet, KFWGNztruth_uav2, color='blue', linewidth=2, label='Ground truth coordinate, UAV 2')
axs12[2].axhline(y=KFWGNztruth_uav2[-1], color='green', linestyle='--', alpha=0.5)
axs12[2].set_ylabel("Z-axis")

for ax in axs12:
    ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
    ax.legend(loc='upper right', facecolor='#fafafa')
    ax.set_facecolor('#fcfcfc')
plt.tight_layout()

plt.savefig('../plots/KF_WGN_TelemetryBenchmarkUAV2.png', dpi=300, bbox_inches='tight')
plt.show()







