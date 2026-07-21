import numpy as np
import matplotlib.pyplot as plt
import subprocess

print("Performing calculations ...")
subprocess.run(["./NumericalMethodCalcs_test.exe"], check="True")
print("Done. Plotting...")

data_NR = np.loadtxt("NumMethodData_NR.csv",delimiter=",",skiprows=1)
iter_list = data_NR[:,0]
x_list = data_NR[:,1]
plt.plot(iter_list,x_list)
plt.title("Newton-Raphson")
plt.xlabel("iterations")
plt.ylabel("x-values")
plt.grid(True)
plt.show()

data_Euler = np.loadtxt("NumMethodData_Euler.csv",delimiter=",",skiprows=1)
x_list = data_Euler[:,0]
y_list = data_Euler[:,1]
plt.plot(x_list,y_list)
plt.title("Euler's method")
plt.xlabel("x-values")
plt.ylabel("y-values")
plt.grid(True)
plt.show()

data_RK4= np.loadtxt("NumMethodData_RK4.csv",delimiter=",",skiprows=1)
x_list = data_RK4[:,0]
y_list = data_RK4[:,1]
plt.plot(x_list,y_list)
plt.title("Runge-Kutta's method, 4th order")
plt.xlabel("x-values")
plt.ylabel("y-values")
plt.grid(True)
plt.show()

data_Heun= np.loadtxt("NumMethodData_Heun.csv",delimiter=",",skiprows=1)
x_list = data_Heun[:,0]
y_list = data_Heun[:,1]
plt.plot(x_list,y_list)
plt.title("Heun's method")
plt.xlabel("x-values")
plt.ylabel("y-values")
plt.grid(True)
plt.show()