# LQR, LQG Controller Implementation and Analysis for a Non-Linear double Crane System
[![License MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/Prasheel24/two-load-crane/blob/master/License)
---
## Disclaimer
```
This project is shared for informative purposes. 
The original records of this project are present on the University server.
Please use this project for reference only.
```
## Author
Prasheel Renkuntla - [GitHub](https://github.com/Prasheel24)

I am pursuing my Master's in Robotics at the University of Maryland, College Park. My primary area of interest is in Vision integrated Robot Systems.

## Overview

This project deals with stabilizing and controlling a non-linear double crane system. There are two components to the given problem statement for this project. 
* First, The equations of motion for the system are determined. The presented non-linear system is then linearised and an LQR controller is designed to stabilize the obtained linearized system. It is then tuned to stabilize the initial non-linear system.
* Second, For each of the observable output vectors, a Luenberger Observer is designed for the linear system with the given initial conditions and a step response. Finally, an LQG controller is designed to stabilise the non linear system using a Kalman Filter along with the existing LQR control.

## Dependencies

* MATLAB - [Installation](https://www.mathworks.com/downloads/)

## Run
* To check the Controllability - [ControllabilityCheck](https://github.com/Prasheel24/two-load-crane/blob/master/code/ControllabilityCheck.m)

* To run the LQR control for linear system - [LQRControlLinearised](https://github.com/Prasheel24/two-load-crane/blob/master/code/LQRControlLinearised.m)
* To run the LQR control for Non linear system - [LQRControlNonLinear](https://github.com/Prasheel24/two-load-crane/blob/master/code/LQRControlNonLinear.m)

* To check the Observability - [ObservabilityCheck](https://github.com/Prasheel24/two-load-crane/blob/master/code/ObservabilityCheck.m)

* The Lyapunov's indirect method can be run with this file - [Lyapunov](https://github.com/Prasheel24/two-load-crane/blob/master/code/LyapunovIndirectMethod.m)

* Luenberger observers for different output vectors are also provided.

* LQG control for Linear system can be simulated using this file - [LQGControlLinear](https://github.com/Prasheel24/two-load-crane/blob/master/code/LQGControlLinear.m)

The rest of the controllers are built in Simulink and their simulation results can be verified.

## Demo
The output folder consists of both the LQR and LQG controller resultant graphs when subjected to initial conditions and step response for both the Non linear and Linear system. Following figure shows the LQR controller for Linear system under initial conditions - 
<p align="center">
<h5>LQR under initial conditions</h5>
<img src="/output/LQRInit.PNG" width="70%">
</p>
Graph below shows the LQR control for Non linear system -
<p align="center">
<h5>LQR Control for Non Linear System</h5>
<img src="/output/LQRNonLinear.PNG" width="70%">
</p>
The Luenberger observer for x(t) output vector with step input is shown below - 
<p align="center">
<h5>Luenberger Observer output for x(t) with step input</h5>
<img src="/output/LuenXStep.PNG" width="70%">
</p>
LQG controller output for x(t) is given below(Linear System) -
<p align="center">
<h5>LQG Control for x(t) in Linear</h5>
<img src="/output/xLinearLQGGraph.PNG" width="70%">
</p>
FInally, the graph below shows the LQG controller for non linear system given the output vector to be x(t) -
<p align="center">
<h5>LQG Controller for Non linear system - x(t)</h5>
<img src="/output/xGraph.PNG" width="70%">
</p>

## License 
```
MIT License

Copyright (c) 2020 Prasheel Renkuntla

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
## References
* E. V. Kumar and J. Jerome. Robust lqr controller design for stabilizing and trajectory tracking of inverted pendulum. Procedia Engineering, 64:169-178, 2013.
* Linearisation - [Link](http://en.wikipedia.org/wiki/Linearization)
* Luenberger Observer - [Link](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic%E2%80%93Gaussian_control)
* LQG Controller - [Link](http://en.wikipedia.org/wiki/State_observer)
