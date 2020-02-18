# Simulation of point to point reaching using a two link human arm model controlled by an iLQR controller

Worked on this as a part of the project for the optimal control course which I took in 2017 (All the code files are from 2017)

The following articles were referred to complete this project.

(1) [Li and Todorov, 2004](https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf)

(2) [Tassa et al., 2014](https://homes.cs.washington.edu/~todorov/papers/TassaICRA14.pdf)

## Model and task
1. The model was generated based on the parameters used in (1), excluding the muscle models and controlling joint torques directly.
2. Four targets were placed at different locations on the 2D plane and reaching to these targets from a common starting target were simulated.

## Initialization of control
The initialization of control actions at each time step was obtained by modeling the problem as an infinite horizon LQR. The reaching trajectories due to this control are unsatisfactory as shown in the [initial animation](https://github.com/Rakshith6/ILQR_TwoLinkArm_Reaching/blob/master/AnimateLinksInitial.mp4.avi) and figure below (red trajectories). 

## ILQR controller 
The controller actions were optimized iteratively using methods described in (2). The final control solutions shown in the figure below and animated in [final animation](https://github.com/Rakshith6/ILQR_TwoLinkArm_Reaching/blob/master/AnimateLinksFinal.mp4.avi) were reached under 10 iterations for all targets. Trajectories from selected iterations are shown in grey (dark to light in increasing iteration number).

![](https://github.com/Rakshith6/ILQR_TwoLinkArm_Reaching/blob/master/Trajectory_AllTargets.png)

