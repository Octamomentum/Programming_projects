# Single-Target UAV Tracking (STUT) - Beta update

This project implements a modular object-oriented radar system in C++ for detection and tracking of UAVs.

The project models UAV tracking as a dynamic data assimilation problem using a generated radar environment consisting of landmark locations and UAV sensor coordinates. 

The radar system was developed as an independent R&D project with focus on:

- Object-oriented software design
- Software robustness and memory management
- Sensor data processing
- Numerical estimation
- Testing and verification

##  Motivation

The idea of the project was to investigate how radar measurements can be used to estimate UAV positions under the influence of potential noisy sensor data sources.

Furthermore, the project explores the effectiveness of different numerical estimation strategies for solving the single-target UAV tracking (STUT) problem.

The project combines mathematical modelling, numerical methods, noisy sensor data treatment and software engineering to explore the challenges of single-target UAV tracking in sensor-based systems.


## Technical Features

The project includes:

- Pseudo-randomized radar environment including landmark locations and sensor data generation
- UAV target tracking
- Sensor data estimation using implemented numerical methods
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

Responsible for the generation of potential noisy sensor data sources for true distances of the moving UAVs relative to landmark locations, using White Gaussian Noise. It is designed for choosing the option to either generate the noisy data or to keep the STUT problem ideal without affected noise.

### TrackingParameters

Stores and manages tracking-related data between states. 

### MatrixCalculations

Responsible for the entire dynamical data assimilation framework and applies numerical methods to solve least-square problems.

### UAVTracking

The main class that initializes a generated radar environment from generateRadarStructure and applies the chosen numerical method strategy to collect the results which later are written to csv-files. 

### UAVTracking_parameters

Represents the needed parameters for the STUT problem.

### plots_UAVTracking

A script in Python that uses data processing pipeline to visualize the estimated results.

### Test Suites

Verification framework based on GoogleTest.

## Mathematical Background

The Single-Target UAV Tracking (STUT) problem is formulated as a dynamical data assimilation problem. 

The radar environment follows a spherical coordinate system centered at the origin.

Radar observations are either ideal or affected by potential noisy data sources using White Gaussian Noise to generate real-world sensor uncertainty  relating to true distances of the UAVs relative to landmarks. 

Estimated positions are calculated through nonlinear least-squares optimization using an implemented numerical method strategy: Gauss-Newton or Levenberg-Marquardt.

The goal of this mathematical framework is to study estimation performance behavior under real-world physical constraints.

##  Verification & Testing

This project was verified using: 

- Unit tests implemented with GoogleTest
- Dynamic memory analysis using AddressSanitizer
- Validation of estimation results through numerical experiments

The project has been continuously tested during development to ensure robust and efficient software architecture. 

## Experimental Results

Experiments were designed to evaluate estimation performance behavior under situational UAV tracking problems: ideal vs uncertain sensor data.

The results show that:

- The estimator performs reliably under the ideal case. In the uncertainty case, however, the reliability tends to decrease as the noise level increases with respect to the variance relating to the true distances of the UAVs relative to landmark locations. 
- Strong wind disturbances and higher UAV velocity magnitudes increase the risk of numerical instability.
- The Gauss-Newton method remains effective when provided with initial guesses that are reasonably far from the moving UAVs in the radar. 
- The Levenberg-Marquardt method provided the most robust performance among the tested estimation strategies, including solving the STUT problem with initial guesses that are extremely far from the radar. 
- In the uncertainty case, Gauss-Newton struggles to converge as the norm for the directional derivative isn't minimal, indicating that the estimate cannot reach closer to the landmarks in the radar. Levenberg-Marquardt tends to, however, converge within a few iterations for a sufficient initialized regularization parameter lambda.

##  Future Improvements

Possible improvements for future updates include:

- Kalman filtering techniques
- Similar numerical methods, for instance L-BFGS
- Multi-target (fleet of UAVs) tracking
- Real radar data integration
- Battery depletion modelling
- Alternative kinematic models
- Armijo line search for damped least-squares problems

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
- Git

### Numerical Methods & Optimization

- Gauss-Newton
- Levenberg-Marquardt
- Nonlinear Least-Squares Optimization
- QR-decomposition
- Givens Matrices
- Line Search (for the kinematic UAV movement prediction)

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
