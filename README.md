# Introduction
----------
''Virtual-Cycling-Simulator'' is a project to simulate virtual cycling environment. 
In the virtual cycling environment, cycling can be done even without a cycling machine.
In hardware level, the virtual cycling environment can be transformed by exoskeleton.

# Features
--------

- A human dynamics model, exoskeleton dynamics model, as well as a bicycle dynamcis model is created. 
- human model and exoskeleton model are created and wrote in C code verison using ''SymPy'' and ''Pydy''
- Cycling model is created in python language
- Direct collocaiton format optimization structure code is provided

For more detail of how these model be created, please look at the Wiki of this project.


# Dependencies
===========
- numpy  >= 1.14.3     
- pydy  >= 0.4.0
- scipy >= 1.1.0
- sympy  >= 1.1.1
- cython >= 0.28.2
- ipopt
- pyipopt

# Usage

There are two ways of using this simulator: forward simulation and trajectory optimization. 

Both of these two ways are included in example. Please check them to understand how to use the model.

