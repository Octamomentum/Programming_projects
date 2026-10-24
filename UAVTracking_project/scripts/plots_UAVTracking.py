#This file is meant for plotting the results generated from C++ code 

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import LogFormatterMathtext
import subprocess
import sys

print("Compiling C++ project UAV Tracking...")

subprocess.run(["cmake", "--build", "../build", "--config", "Release"], check="True")

print("Plotting the results...")

subprocess.run(["../build/main_run.exe"],check="True")

#Getting needed parameters for the plots

UAVParameters = np.loadtxt("../output_data/UAVParameters.csv",delimiter=",",skiprows=1)
formatter = LogFormatterMathtext()
num_of_landmarks = formatter(UAVParameters[0])
num_of_uavs = int(UAVParameters[1])
radar_radius = formatter(UAVParameters[2])
uav_guess_radius = formatter(UAVParameters[3])
variance = formatter(UAVParameters[4])



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
KFWGNdata_iter = KFWGNdata_L2[:,0]
KFWGNdata_L2Error = KFWGNdata_L2[:,1]


plt.suptitle(rf"L2 Convergence solution for {num_of_uavs} UAVs: {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}",
              fontsize=12, fontweight='bold')
plt.plot(GNnoWGNdata_iter,GNnoWGNdata_L2Error, label='Gauss-Newton')
plt.plot(LMnoWGNdata_iter,LMnoWGNdata_L2Error, color = 'g', marker = 'o',linestyle='-',label='Levenberg-Marquardt')
plt.plot(KFnoWGNdata_iter,KFnoWGNdata_L2Error, color = 'r', marker = 's',linestyle='--',label='Kalman Filtering')
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.legend(loc='best')
plt.tight_layout()
plt.savefig('../plots/L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.show()

plt.suptitle(rf"L2 Convergence solution for {num_of_uavs} UAVs, WGN: {num_of_landmarks} Landmarks,  $r_{{radar}}$ = {radar_radius}, $r_{{guess}}$ = {uav_guess_radius},$\sigma^2$ = {variance}",
              fontsize=12, fontweight='bold')
plt.plot(GNWGNdata_iter,GNWGNdata_L2Error, label='Gauss-Newton')
plt.plot(LMWGNdata_iter,LMWGNdata_L2Error, color = 'g', marker = 'o',linestyle='-',label='Levenberg-Marquardt')
plt.plot(KFWGNdata_iter,KFWGNdata_L2Error, color = 'r', marker = 's',linestyle='--',label='Kalman Filtering')
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.legend(loc='best')
plt.tight_layout()
plt.savefig('../plots/WGN_L2ConvergenceSolution.png', dpi=300, bbox_inches='tight')
plt.legend()
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


plt.suptitle(rf"L2 Convergence residual for {num_of_uavs} UAVs: {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius}, $r_{{guess}}$ = {uav_guess_radius}",
              fontsize=12, fontweight='bold')
plt.plot(GNnoWGNdata_iterRes,GNnoWGNdata_L2ErrorRes, label='Gauss-Newton')
plt.plot(LMnoWGNdata_iter,LMnoWGNdata_L2ErrorRes, color = 'g', marker = 'o',linestyle='-',label='Levenberg-Marquardt')
plt.plot(KFnoWGNdata_iter,KFnoWGNdata_L2ErrorRes, color = 'r', marker = 's',linestyle='--',label='Kalman Filtering')
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.legend(loc='best')
plt.tight_layout()
plt.savefig('../plots/L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.legend()
plt.show()

plt.suptitle(rf"L2 Convergence residual for {num_of_uavs} UAVs, WGN: {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}, $\sigma^2$ = {variance}",
             fontsize=12, fontweight='bold')
plt.plot(GNWGNdata_iterRes,GNWGNdata_L2ErrorRes, label='Gauss-Newton')
plt.plot(LMWGNdata_iterRes,LMWGNdata_L2ErrorRes, color = 'g', marker = 'o',linestyle='-',label='Levenberg-Marquardt')
plt.plot(KFWGNdata_iterRes,KFWGNdata_L2ErrorRes, color = 'r', marker = 's',linestyle='--',label='Kalman Filtering')
plt.xlabel("Iterations")
plt.ylabel("L2 Norm")
plt.legend(loc='best')
plt.tight_layout()
plt.savefig('../plots/WGN_L2ConvergenceResidual.png', dpi=300, bbox_inches='tight')
plt.legend()
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


plt.suptitle(rf"Elapsed Time for {num_of_uavs} UAVs: {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}", fontsize=12,
              fontweight='bold')
plt.plot(GNnoWGNdata_iterTime,GNnoWGNdata_errorTime, label='Gauss-Newton')
plt.plot(LMnoWGNdata_iterTime,LMnoWGNdata_errorTime, color = 'g', marker = 'o',linestyle='-',label='Levenberg-Marquardt')
plt.plot(KFnoWGNdata_iterTime,KFnoWGNdata_errorTime, color = 'r', marker = 's',linestyle='--',label='Kalman Filtering')
plt.xlabel("Iterations")
plt.ylabel("Time")
plt.legend(loc='best')
plt.tight_layout()
plt.savefig('../plots/ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.legend()
plt.show()

plt.suptitle(rf" Elapsed Time for {num_of_uavs} UAVs, WGN: {num_of_landmarks} Landmarks,  $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}, $\sigma^2$ = {variance}",
             fontsize=12, fontweight='bold')
plt.plot(GNWGNdata_iterTime,GNWGNdata_errorTime, label='Gauss-Newton')
plt.plot(LMWGNdata_iterTime,LMWGNdata_errorTime, color = 'g', marker = 'o',linestyle='-',label='Levenberg-Marquardt')
plt.plot(KFWGNdata_iterTime,KFWGNdata_errorTime, color = 'r', marker = 's',linestyle='--',label='Kalman Filtering')
plt.xlabel("Iterations")
plt.ylabel("Time")
plt.legend(loc='best')
plt.tight_layout()
plt.savefig('../plots/WGN_ElapsedTime.png', dpi=300, bbox_inches='tight')
plt.legend()
plt.show()


#Method that generates a list of lists for the following datasets below
def generateUAVData(lst, data_lst, n):

    for i in range(0,n):
        uav_list = []
        for j in range(3*i+1,3*i+4):
            uav_list.append(data_lst[:,j])
    
        lst.append(uav_list)


#Fourth file is data that represents the moving ground truth coordinates for each iteration

GNnoWGNdatatruth = np.loadtxt("../output_data/MovingUAV_GN,noWGN.csv",delimiter=",",skiprows=1)
GNnoWGNdataiter = GNnoWGNdatatruth[:,0]

GNnoWGN_Truth = []
generateUAVData(GNnoWGN_Truth,GNnoWGNdatatruth,num_of_uavs)

GNWGNdatatruth = np.loadtxt("../output_data/MovingUAV_GN,WGN.csv",delimiter=",",skiprows=1)
GNWGNdataiter = GNWGNdatatruth[:,0]

GNWGN_Truth = []
generateUAVData(GNWGN_Truth,GNWGNdatatruth,num_of_uavs)


LMnoWGNdatatruth = np.loadtxt("../output_data/MovingUAV_LM,noWGN.csv",delimiter=",",skiprows=1)
LMnoWGNdataiter = LMnoWGNdatatruth[:,0]

LMnoWGN_Truth = []
generateUAVData(LMnoWGN_Truth,LMnoWGNdatatruth,num_of_uavs)

LMWGNdatatruth = np.loadtxt("../output_data/MovingUAV_LM,WGN.csv",delimiter=",",skiprows=1)
LMWGNdataiter = LMWGNdatatruth[:,0]

LMWGN_Truth = []
generateUAVData(LMWGN_Truth,LMWGNdatatruth,num_of_uavs)

KFnoWGNdatatruth = np.loadtxt("../output_data/MovingUAV_KALMANFILTER,noWGN.csv",delimiter=",",skiprows=1)
KFnoWGNdataiter = KFnoWGNdatatruth[:,0]

KFnoWGN_Truth = []
generateUAVData(KFnoWGN_Truth,KFnoWGNdatatruth,num_of_uavs)

KFWGNdatatruth = np.loadtxt("../output_data/MovingUAV_KALMANFILTER,WGN.csv",delimiter=",",skiprows=1)
KFWGNdataiter = KFWGNdatatruth[:,0]

KFWGN_Truth = []
generateUAVData(KFWGN_Truth,KFWGNdatatruth,num_of_uavs)

#The fifth file is data that represents the estimated sensor coordinates for each iteration
GNnoWGNdataest = np.loadtxt("../output_data/UAVConvergence_GN,noWGN.csv",delimiter=",",skiprows=1)
GNnoWGNdataiter_est = GNnoWGNdataest[:,0]

GNnoWGN_Guess = []
generateUAVData(GNnoWGN_Guess,GNnoWGNdataest,num_of_uavs)

GNWGNdataest = np.loadtxt("../output_data/UAVConvergence_GN,WGN.csv",delimiter=",",skiprows=1)
GNWGNdataiter_est = GNWGNdataest[:,0]

GNWGN_Guess = []
generateUAVData(GNWGN_Guess,GNWGNdataest,num_of_uavs)

LMnoWGNdataest = np.loadtxt("../output_data/UAVConvergence_LM,noWGN.csv",delimiter=",",skiprows=1)
LMnoWGNdataiter_est = LMnoWGNdataest[:,0]

LMnoWGN_Guess = []
generateUAVData(LMnoWGN_Guess,LMnoWGNdataest,num_of_uavs)

LMWGNdataest = np.loadtxt("../output_data/UAVConvergence_LM,WGN.csv",delimiter=",",skiprows=1)
LMWGNdataiter_est = LMWGNdataest[:,0]

LMWGN_Guess = []
generateUAVData(LMWGN_Guess,LMWGNdataest,num_of_uavs)

KFnoWGNdataest = np.loadtxt("../output_data/UAVConvergence_KALMANFILTER,noWGN.csv",delimiter=",",skiprows=1)
KFnoWGNdataiter_est = KFnoWGNdataest[:,0]

KFnoWGN_Guess = []
generateUAVData(KFnoWGN_Guess,KFnoWGNdataest,num_of_uavs)

KFWGNdataest = np.loadtxt("../output_data/UAVConvergence_KALMANFILTER,WGN.csv",delimiter=",",skiprows=1)
KFWGNdataiter_est = KFWGNdataest[:,0]

KFWGN_Guess = []
generateUAVData(KFWGN_Guess,KFWGNdataest,num_of_uavs)

#Methods that plot Telemetry Benchmarks for the 3D coordinates in comparison between moving UAVs and estimates

def plotTelemetryBenchmark(iterTruth,iterEst,coordTruth,coordEst,Estmethod,n):
    for i in range(0,n):
        fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        fig.suptitle(rf"UAV Tracking {Estmethod}, {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}: UAV {i+1}",
                      fontsize=12, fontweight='bold')
        TruthcoordX = coordTruth[i][0]
        TruthcoordY = coordTruth[i][1]
        TruthcoordZ = coordTruth[i][2]
        EstcoordX = coordEst[i][0]
        EstcoordY = coordEst[i][1]
        EstcoordZ = coordEst[i][2]

        # Panel for x-axis
        axs[0].plot(iterEst, EstcoordX, marker = 'o',linestyle='-', color='red', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[0].plot(iterTruth, TruthcoordX, color='blue', linewidth=2, label =rf"Ground truth coordinate, UAV {i+1}")
        axs[0].axhline(y=TruthcoordX[-1], color='green', linestyle='--', label = rf"Final Ground truth coordinate, UAV {i+1}", alpha=0.5)
        axs[0].set_ylabel("X-axis")

        # Panel for y-axis
        axs[1].plot(iterEst, EstcoordY, marker = 'o',linestyle='-', color='red', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[1].plot(iterTruth, TruthcoordY, color='blue', linewidth=2, label=rf'Ground truth coordinate, UAV {i+1}')
        axs[1].axhline(y=TruthcoordY[-1], color='green', linestyle='--',label = rf"Final Ground truth coordinate, UAV {i+1}", alpha=0.5)
        axs[1].set_ylabel("Y-axis")

        # Panel for z-axis
        axs[2].plot(iterEst, EstcoordZ, marker = 'o',linestyle='-', color='red', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[2].plot(iterTruth, TruthcoordZ, color='blue', linewidth=2, label=rf'Ground truth coordinate, UAV {i+1}')
        axs[2].axhline(y=TruthcoordZ[-1], color='green', linestyle='--',label = rf"Final Ground truth coordinate, UAV {i+1}", alpha=0.5)
        axs[2].set_ylabel("Z-axis")

        for ax in axs:
            ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
            ax.legend(loc='upper right', facecolor='#fafafa')
            ax.set_facecolor('#fcfcfc')
        plt.tight_layout()

        plt.savefig(rf'../plots/{Estmethod}_TelemetryBenchmarkUAV{i+1}.png', dpi=300, bbox_inches='tight')
        plt.show()


def plotTelemetryBenchmark_WGN(iterTruth,iterEst,coordTruth,coordEst,Estmethod,n):
    for i in range(0,n):
        fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        fig.suptitle(rf"UAV Tracking {Estmethod} WGN, {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}, $\sigma^2$ = {variance}: UAV {i+1}",
                      fontsize=12, fontweight='bold')
        TruthcoordX = coordTruth[i][0]
        TruthcoordY = coordTruth[i][1]
        TruthcoordZ = coordTruth[i][2]
        EstcoordX = coordEst[i][0]
        EstcoordY = coordEst[i][1]
        EstcoordZ = coordEst[i][2]

        # Panel for x-axis
        axs[0].plot(iterEst, EstcoordX, marker = 'o',linestyle='-', color='red', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[0].plot(iterTruth, TruthcoordX, color='blue', linewidth=2, label =rf"Ground truth coordinate, UAV {i+1}")
        axs[0].axhline(y=TruthcoordX[-1], color='green', linestyle='--',label = rf"Final Ground truth coordinate, UAV {i+1}", alpha=0.5)
        axs[0].set_ylabel("X-axis")

        # Panel for y-axis
        axs[1].plot(iterEst, EstcoordY, marker = 'o',linestyle='-', color='red', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[1].plot(iterTruth, TruthcoordY, color='blue', linewidth=2, label=rf'Ground truth coordinate, UAV {i+1}')
        axs[1].axhline(y=TruthcoordY[-1], color='green', linestyle='--',label = rf"Final Ground truth coordinate, UAV {i+1}", alpha=0.5)
        axs[1].set_ylabel("Y-axis")

        # Panel for z-axis
        axs[2].plot(iterEst, EstcoordZ, marker = 'o',linestyle='-', color='red', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[2].plot(iterTruth, TruthcoordZ, color='blue', linewidth=2, label=rf'Ground truth coordinate, UAV {i+1}')
        axs[2].axhline(y=TruthcoordZ[-1], color='green', linestyle='--',label = rf"Final Ground truth coordinate, UAV {i+1}", alpha=0.5)
        axs[2].set_ylabel("Z-axis")

        for ax in axs:
            ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
            ax.legend(loc='upper right', facecolor='#fafafa')
            ax.set_facecolor('#fcfcfc')
        plt.tight_layout()

        plt.savefig(rf'../plots/{Estmethod}_WGN_TelemetryBenchmarkUAV{i+1}.png', dpi=300, bbox_inches='tight')
        plt.show()


plotTelemetryBenchmark(GNnoWGNdata_iter,GNnoWGNdataiter_est,GNnoWGN_Truth,GNnoWGN_Guess,"GaussNewton",num_of_uavs)
plotTelemetryBenchmark_WGN(GNWGNdata_iter,GNWGNdataiter_est,GNWGN_Truth,GNWGN_Guess,"GaussNewton",num_of_uavs)
plotTelemetryBenchmark(LMnoWGNdata_iter,LMnoWGNdataiter_est,LMnoWGN_Truth,LMnoWGN_Guess,"LevenbergMarquardt",num_of_uavs)
plotTelemetryBenchmark_WGN(LMWGNdata_iter,LMWGNdataiter_est,LMWGN_Truth,LMWGN_Guess,"LevenbergMarquardt",num_of_uavs)
plotTelemetryBenchmark(KFnoWGNdata_iter,KFnoWGNdataiter_est,KFnoWGN_Truth,KFnoWGN_Guess,"KalmanFilter",num_of_uavs)
plotTelemetryBenchmark_WGN(KFWGNdata_iter,KFWGNdataiter_est,KFWGN_Truth,KFWGN_Guess,"KalmanFilter",num_of_uavs)









