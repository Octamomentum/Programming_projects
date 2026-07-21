import numpy as np
import matplotlib.pyplot as plt
import subprocess

print("Calculations in C++ going on...")
subprocess.run(["./run_test.exe"], check="True")
print("Done. Reading files...")

data_NR = np.loadtxt("SimData_NR.csv",delimiter=",",skiprows=1)
iter_NR = data_NR[:,0]
x_NR = data_NR[:,1]
plt.plot(iter_NR,x_NR)
plt.title("Newton-Raphson")
plt.xlabel("iterations")
plt.ylabel("x solution")
plt.grid(True)
plt.show()

data_Euler = np.loadtxt("SimData_Euler.csv",delimiter=",",skiprows=1)
x_Euler = data_Euler[:,0]
y_Euler = data_Euler[:,1]
plt.plot(x_Euler,y_Euler)
plt.title("Euler Forward")
plt.xlabel("x")
plt.ylabel("f(x,y)")
plt.grid(True)
plt.show()

data_RK4 = np.loadtxt("SimData_RK4.csv",delimiter=",",skiprows=1)
x_RK4 = data_RK4[:,0]
y_RK4 = data_RK4[:,1]
plt.plot(x_RK4,y_RK4)
plt.title("Runge-Kutta")
plt.xlabel("x")
plt.ylabel("f(x,y)")
plt.grid(True)
plt.show()

data_Heun = np.loadtxt("SimData_Heun.csv",delimiter=",",skiprows=1)
x_Heun = data_Heun[:,0]
y_Heun = data_Heun[:,1]
plt.plot(x_Heun,y_Heun)
plt.title("Heun")
plt.xlabel("x")
plt.ylabel("f(x,y)")
plt.grid(True)
plt.show()
