#Python script for automatically building C++ project and visualizing numerical results 

import numpy as np
import matplotlib.pyplot as plt
import subprocess

print("Compiling C++ project UAV Tracking...")

subprocess.run(["cmake", "--build", "../build", "--config", "Release"], check="True")

subprocess.run(["../build/main_run.exe"],check="True")

print("Plotting the results...")

#Getting needed parameters for the plots

UAVParameters = np.loadtxt("../output_data/UAVParameters.csv",delimiter=",",skiprows=1)
num_of_landmarks = int(UAVParameters[0])
num_of_uavs = int(UAVParameters[1])
radar_radius = int(UAVParameters[2])
uav_guess_radius = int(UAVParameters[3])
variance = UAVParameters[4]
gamma = UAVParameters[5]

#First file is data for the L2-measures of the step solution
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


plt.suptitle(rf"L2 Convergence solution for {num_of_uavs} UAVs: {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}, $\gamma$ = {gamma}",
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

#The second file is data for the L2-measures of the residual distances relative to the landmarks
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


plt.suptitle(rf"L2 Convergence residual for {num_of_uavs} UAVs: {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius}, $r_{{guess}}$ = {uav_guess_radius}, $\gamma$ = {gamma}",
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

#The third file is data representing elapsed time using the numerical estimation methods over iterations

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


plt.suptitle(rf"Elapsed Time for {num_of_uavs} UAVs: {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}, $\gamma$ = {gamma}", fontsize=12,
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

plt.suptitle(rf" Elapsed Time for {num_of_uavs} UAVs, WGN: {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}, $\sigma^2$ = {variance}",
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

#The fourth file is data that represents the estimated UAV coordinates for each state
GNnoWGNdataest = np.loadtxt("../output_data/UAVEstimates_GN,noWGN.csv",delimiter=",",skiprows=1)
GNnoWGNdataiter_est = GNnoWGNdataest[:,0]

GNnoWGN_Guess = []
generateUAVData(GNnoWGN_Guess,GNnoWGNdataest,num_of_uavs)

GNWGNdataest = np.loadtxt("../output_data/UAVEstimates_GN,WGN.csv",delimiter=",",skiprows=1)
GNWGNdataiter_est = GNWGNdataest[:,0]

GNWGN_Guess = []
generateUAVData(GNWGN_Guess,GNWGNdataest,num_of_uavs)

LMnoWGNdataest = np.loadtxt("../output_data/UAVEstimates_LM,noWGN.csv",delimiter=",",skiprows=1)
LMnoWGNdataiter_est = LMnoWGNdataest[:,0]

LMnoWGN_Guess = []
generateUAVData(LMnoWGN_Guess,LMnoWGNdataest,num_of_uavs)

LMWGNdataest = np.loadtxt("../output_data/UAVEstimates_LM,WGN.csv",delimiter=",",skiprows=1)
LMWGNdataiter_est = LMWGNdataest[:,0]

LMWGN_Guess = []
generateUAVData(LMWGN_Guess,LMWGNdataest,num_of_uavs)

KFnoWGNdataest = np.loadtxt("../output_data/UAVEstimates_KALMANFILTER,noWGN.csv",delimiter=",",skiprows=1)
KFnoWGNdataiter_est = KFnoWGNdataest[:,0]

KFnoWGN_Guess = []
generateUAVData(KFnoWGN_Guess,KFnoWGNdataest,num_of_uavs)

KFWGNdataest = np.loadtxt("../output_data/UAVEstimates_KALMANFILTER,WGN.csv",delimiter=",",skiprows=1)
KFWGNdataiter_est = KFWGNdataest[:,0]

KFWGN_Guess = []
generateUAVData(KFWGN_Guess,KFWGNdataest,num_of_uavs)

#The fifth file is data for the radar anchor points

def getRadarAnchors(lst,RadarAnchors,n):

    for i in range(0,n):
        RadarAnchorList = []
        for j in range(3*i,3*i + 3):
            RadarAnchorList.append(RadarAnchors[j])
        lst.append(RadarAnchorList)
    return lst 

RadarAnchorData = np.loadtxt("../output_data/RadarAnchors.csv",delimiter=",",skiprows=1)
RadarAnchorList = []
RadarAnchorLocations = getRadarAnchors(RadarAnchorList,RadarAnchorData,num_of_uavs)

#Methods that plot convergence benchmarks for the esimated UAV coordinates

def plotEstimationBenchmark(iterEst,coordEst,RadarAnchors,Estmethod,n):
    for i in range(0,n):
        fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        fig.suptitle(rf"UAV Tracking {Estmethod}, {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}, $\gamma$ = {gamma}: UAV {i+1}",
                      fontsize=12, fontweight='bold')
        RAcoordX = RadarAnchors[i][0]
        RAcoordY = RadarAnchors[i][1]
        RAcoordZ = RadarAnchors[i][2]
        EstcoordX = coordEst[i][0]
        EstcoordY = coordEst[i][1]
        EstcoordZ = coordEst[i][2]

        # Panel for x-axis
        axs[0].plot(iterEst, EstcoordX, marker = 'o',linestyle='-', color='blue', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[0].axhline(y=RAcoordX, color='red', linestyle='--', label = rf"Radar Anchor coordinate", alpha=0.5)
        axs[0].set_ylabel("X-axis")

        # Panel for y-axis
        axs[1].plot(iterEst, EstcoordY, marker = 'o',linestyle='-', color='blue', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[1].axhline(y=RAcoordY, color='red', linestyle='--',label = rf"Radar Anchor coordinate", alpha=0.5)
        axs[1].set_ylabel("Y-axis")

        # Panel for z-axis
        axs[2].plot(iterEst, EstcoordZ, marker = 'o',linestyle='-', color='blue', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[2].axhline(y=RAcoordZ, color='red', linestyle='--',label = rf"Radar Anchor coordinate", alpha=0.5)
        axs[2].set_ylabel("Z-axis")

        for ax in axs:
            ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
            ax.legend(loc='upper right', facecolor='#fafafa')
            ax.set_facecolor('#fcfcfc')
        plt.tight_layout()

        plt.savefig(rf'../plots/{Estmethod}_EstimationBenchmarkUAV{i+1}.png', dpi=300, bbox_inches='tight')
        plt.show()


def plotEstimationBenchmark_WGN(iterEst,coordEst,RadarAnchors,Estmethod,n):
    for i in range(0,n):
        fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        fig.suptitle(rf"UAV Tracking {Estmethod} WGN, {num_of_landmarks} Landmarks, $r_{{radar}}$ = {radar_radius},$r_{{guess}}$ = {uav_guess_radius}, $\sigma^2$ = {variance}: UAV {i+1}",
                      fontsize=12, fontweight='bold')
        RAcoordX = RadarAnchors[i][0]
        RAcoordY = RadarAnchors[i][1]
        RAcoordZ = RadarAnchors[i][2]
        EstcoordX = coordEst[i][0]
        EstcoordY = coordEst[i][1]
        EstcoordZ = coordEst[i][2]
        
        # Panel for x-axis
        axs[0].plot(iterEst, EstcoordX, marker = 'o',linestyle='-', color='blue', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[0].axhline(y=RAcoordX, color='red', linestyle='--', label = rf"Radar Anchor coordinate", alpha=0.5)
        axs[0].set_ylabel("X-axis")
        
        # Panel for y-axis
        axs[1].plot(iterEst, EstcoordY, marker = 'o',linestyle='-', color='blue', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[1].axhline(y=RAcoordY, color='red', linestyle='--',label = rf"Radar Anchor coordinate", alpha=0.5)
        axs[1].set_ylabel("Y-axis")
        
                # Panel for z-axis
        axs[2].plot(iterEst, EstcoordZ, marker = 'o',linestyle='-', color='blue', linewidth=2, label=rf'Estimated coordinate, UAV {i+1}')
        axs[2].axhline(y=RAcoordZ, color='red', linestyle='--',label = rf"Radar Anchor coordinate", alpha=0.5)
        axs[2].set_ylabel("Z-axis")

        for ax in axs:
            ax.grid(True, which="both", linestyle=":", alpha=0.6, color='gray')
            ax.legend(loc='upper right', facecolor='#fafafa')
            ax.set_facecolor('#fcfcfc')
        plt.tight_layout()

        plt.savefig(rf'../plots/{Estmethod}_WGN_EstimationBenchmarkUAV{i+1}.png', dpi=300, bbox_inches='tight')
        plt.show()


plotEstimationBenchmark(GNnoWGNdata_iter,GNnoWGN_Guess,RadarAnchorLocations,"Gauss-Newton",num_of_uavs)
plotEstimationBenchmark_WGN(GNWGNdata_iter,GNWGN_Guess,RadarAnchorLocations,"Gauss-Newton",num_of_uavs)
plotEstimationBenchmark(LMnoWGNdata_iter,LMnoWGN_Guess,RadarAnchorLocations,"Levenberg-Marquardt",num_of_uavs)
plotEstimationBenchmark_WGN(LMWGNdata_iter,LMWGN_Guess,RadarAnchorLocations,"Levenberg-Marquardt",num_of_uavs)
plotEstimationBenchmark(KFnoWGNdata_iter,KFnoWGN_Guess,RadarAnchorLocations,"Kalman Filter",num_of_uavs)
plotEstimationBenchmark_WGN(KFWGNdata_iter,KFWGN_Guess,RadarAnchorLocations,"Kalman Filter",num_of_uavs)









