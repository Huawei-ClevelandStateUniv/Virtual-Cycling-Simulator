# Introduction

''Virtual-Cycling-Simulator'' is a project to simulate virtual cycling environment. 
With the simulator, cycling can be done using an exoskeleton instead of a cycling machine. 
The goal of the virtual cycling simulator is to calcualte the torques which hip/knee joints need to overcome at different cycling speed and loads.
The method of this research can be used in other motions, for instance, running, walking, and even jumpping. 

# Features

- A human dynamics model, exoskeleton dynamics model, as well as a bicycle dynamcis model is created. 
- human model and exoskeleton model are created and wrote in C code verison using ''SymPy'' and ''Pydy''
- Cycling model is created in python language
- Direct collocaiton format optimization structure code is provided

For more detail of how these model be created, please look at the [Wiki](https://github.com/HuaweiWang/Virtual-Cycling-Simulator/wiki) of this project.

# Demo

[![Demo Doccou alpha](https://github.com/HuaweiWang/Virtual-Cycling-Simulator/blob/master/pics/optimize3_result_annimation.gif)](https://youtu.be/zvIn8QWE_BI)

# Dependencies
===========
- numpy  >= 1.14.3     
- pydy  >= 0.4.0
- scipy >= 1.1.0
- sympy  >= 1.1.1
- cython >= 0.28.2
- matplotlib >= 2.2.2
- ipopt
- [cyipopt](https://github.com/matthias-k/cyipopt)

# Usage

There are two ways of using this simulator: forward simulation and trajectory optimization. 

Both of these two ways are included in example. Please check the README in the examples folder.

# Conditions of use

cyipopt is open-source code released under the [EPL] (https://www.eclipse.org/legal/epl-v10.html) license.
