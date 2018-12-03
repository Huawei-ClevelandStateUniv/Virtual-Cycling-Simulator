## Introduction 


This folder includes examples of how to use the Virtual-Cycling-Simulator

### Froward Simulation

Forward simulation is to let human track reference joint trajectories and see if the generated torques can drive the bicycle model cycling at the constant speed or not.

- Joints’ trajectories are got from inverse kinematics
- Local PD controller is used to Let human model track the reference joint angles
- Assume exoskeleton is light weight and can generate target torque with good controller
- No external control of exoskeleton and bicycle model 

### Optimization 

Optimization is to find the minimum torques at all joints that could generate the constant speed cycling.

- Tracking crank trajectory, as well as minimizing joint torques
- No tracking of joints’ trajectories
- Optimization is in direct collocation format
