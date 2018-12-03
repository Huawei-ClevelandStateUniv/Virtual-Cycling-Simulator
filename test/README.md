## Introduction

This folder includes test code of all models and their connection system. 


### test_bicycle_model.py is to check the bicycle model.

It checks two items.

1. forward simulation with the rotation of foot points.
2. derivative check 


### test_human_exoskeleton_model.py is to test the generated and compiled human/exoskeleton model (c code)

Two tests are contained here: leg falling and derivative check

1. leg falling:
    both left and right leg are lifted vertically up initially. Then
    simulation the falling process
    
2. derivative check:
    check the derivative of q, u, a, and F
    

### test_virtual_cycling_simulator.py is to check derivative of the connected models


