# UAV Tracking

This project implements a modular object-oriented radar system in C++ for detection and tracking of moving UAVs.

The project models UAV tracking as a dynamic data assimilation problem using a generated radar environment consisting of landmark locations and UAV coordinates. 

The radar system was developed as an independent R&D project with focus on:

- Object-oriented software design
- Software robustness and memory management
- Sensor data processing
- Numerical estimation
- Testing and verification

##  Motivation

The idea of the project was to investigate how radar measurements can be used to estimate UAV positions under the influence of potential noisy sensor data sources.

Furthermore, the project explores the effectiveness of different numerical estimation strategies for solving the UAV Tracking problem.

The project combines mathematical modelling, numerical estimation methods, noisy sensor data treatment and software engineering to explore the challenges of UAV tracking in sensor-based systems.


## Technical Features

The project includes:

- Pseudo-randomized radar environment including landmark locations and sensor data generation
- Tracking of Multiple UAVs
- Sensor data estimation using implemented numerical estimation methods
- Kinematic UAV movement prediction
- Wind disturbance
- White Gaussian Noise sensor simulation
- Custom generic data structures
- Memory management using smart pointers
- Rule of Five implementation
- GoogleTest unit testing
- AddressSanitizer verification
- Data processing and visualization pipeline in Python

## Software Architecture

The software is divided into several components:

### LinkedListObject

Designed to store data for any data type.

### MatrixObject

Provides matrix storage and numerical operations used throughout the estimation framework.

### generateRadarStructure

Responsible for generating the radar environment by using pseudo-randomized seeds stored in matrix structures.

### PhysicalConstraints

Designed for kinematic movement prediction.

### GenerateWGNSource

Responsible for the generation of potential noisy sensor data sources for true distances of the moving UAVs relative to landmark locations, using White Gaussian Noise. It is designed for choosing the option to either generate the noisy data or to keep the UAV Tracking problem ideal.

### TrackingParameters

Stores and manages tracking-related data between states. 

### MatrixCalculations

Responsible for the mathematical operations underlying the dynamic data assimilation framework.

### GaussNewton

Responsible for state estimation using the Gauss-Newton strategy.

### LevenbergMarquardt

Responsible for state estimation using the Levenberg-Marquardt strategy.

### KalmanFilter

Responsible for state estimation using Kalman filtering techniques.

### StateEstimator

Performs state estimation by applying the selected numerical estimation strategy to the UAV Tracking problem.

### UAVTracking

Responsible for performing the state estimation via StateEstimator in order to collect results that later are written to csv-files.

### MissionUAV_Solver

The main class that initializes a radar environment generated from generateRadarStructure and starts solving the UAV Tracking problem via UAVTracking.

### UAVTracking_parameters

Represents the needed parameters for the UAV Tracking problem.

### main_UAVTracking

The main source file for running the UAV Tracking solver via MissionUAV_Solver.

### plots_UAVTracking

A script in Python that uses a data processing pipeline to automatically build and execute the project, and subsequently generate plots from the UAV Tracking solver results.

### Test Suites

Verification framework based on GoogleTest.

## Usage

### Clone the repository

- git clone https://github.com/Octamomentum/Programming_projects
- cd UAVTracking_project

### Create a build folder and configure the project:

- mkdir build
- cd build
- cmake ..

### Run the Python visualization pipeline

Since the Python script automatically builds and executes the project, it is sufficient to run

- python plots_UAVTracking.py

The script generates the simulation results and visualizes the estimation performance and the estimated UAV trajectories.

## Mathematical Background

The UAV Tracking (UT) problem is formulated as a dynamic data assimilation problem. 

The radar environment follows a spherical coordinate system centered at the origin.

Radar observations are either ideal or affected by potential noisy data sources using White Gaussian Noise to generate real-world sensor uncertainty  relating to true distances of the UAV positions relative to landmark locations. 

Estimated positions are calculated through nonlinear least-squares optimization using an implemented numerical estimation method strategy: Gauss-Newton, Levenberg-Marquardt or an approach using Kalman filtering techniques. 

The UAVs are mutally independent meaning that the covariance matrix P describing uncertainty in the initial states is initialized as the diagonal matrix with diagonal elements as the variance for the maximal positional UAV error offset.

The goal of this mathematical framework is to study estimation performance behavior under real-world physical constraints.

##  Verification & Testing

This project was verified using: 

- Unit tests implemented with GoogleTest
- Dynamic memory analysis using AddressSanitizer
- GitHub Actions via a Continuous Integration (CI) framework
- Validation of estimation results through numerical experiments

The project has been continuously tested during development to ensure robust and efficient software architecture. 

## Experimental Results

Experiments were designed to evaluate estimation performance behavior under situational UAV tracking problems: ideal vs uncertain sensor data.

The results show that:

- The estimator performs reliably under the ideal case. In the uncertainty case, however, the reliability tends to decrease as the noise level increases with respect to the variance relating to the true distances of the UAV positions relative to landmark locations. 
- Strong wind disturbances and higher UAV velocity magnitudes increase the risk of numerical instability.
- The Gauss-Newton method remains effective when provided with initial guesses that are reasonably far from the moving UAVs in the radar. 
- The Levenberg-Marquardt method provided a robust performance based on the Trust-Region approach, tending to converge to the same solution and residual sequence as Gauss-Newton in the ideal case.
- The Kalman filtering method demonstrated the most improved convergence under challenging initialization scenarios and reduced observation sets when sufficiently calibrated. However, divergence may occur for poorly calibrated covariance matrices or strongly overdetermined situations with extreme initialization distances.
- In the uncertainty case, when provided with an additional stop criterion when the absolute error between the previous and current step L2-norm is sufficiently small, all methods converge physically in a neighborhood around the moving UAVs. 

##  Future Improvements

Possible improvements for future updates include:

- Real radar data integration

## Technologies Used

The project was developed using:

### Programming Languages

- Modern C++
- Python

### Software Development

- Object-Oriented Programming
- Rule Of Five
- Smart Pointers
- Custom Generic Data Structures

### Numerical Methods & Optimization

- Gauss-Newton
- Levenberg-Marquardt
- Kalman filtering techniques
- Nonlinear Least-Squares Optimization
- Block/Augmented/Projection matrices
- QR-decomposition
- Givens Matrices
- Tikhonov Regularization for the Kalman filtering method
- Line Search (for the kinematic UAV movement prediction)
- Armijo conditioned line search for the numerical estimations

### Mathematical Concepts

- Data Assimilation
- State Estimation
- White Gaussian Noise
- Mathematical Modelling

### Verification & Testing

- GoogleTest
- AddressSanitizer

### Tools

- CMake
- Git
- GitHub
- GitHub Actions
- Continuous Integration
