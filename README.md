
## Overview

This repository is about using iterative learning control (ILC) for robot reference trajectory tracking problem with the situation of model mismatch. Model Predictive Control (MPC) is also used for comparison. It is initiated by the final project of **MECHENG 599 - Data Driven Methods for Control Systems (2024 Winter)** at University of Michigan - Ann Arbor.

Disclaimer: This is **NOT** a research project. Some part might not be rigorous and suggestions are welcomed.

**ILC and MPC Overview**
<!-- ![ILC Block Diagram](https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/ilc-block-diagram.png) -->
<p align="center">
  <img alt="Image 1" src="https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/ilc-block-diagram.png" width="50%" />
  <img alt="Image 2" src="https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/mpc-block-diagram.png" width="50%" />
</p>

**ILC Initial Iteration and MPC Result**
<!-- ![ilc-qp](https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/trajectory_after_ilc-qp.png) -->
<p align="center">
  <img alt="Image 1" src="https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/trajectory_before_ilc.png" width="45%" />
  <img alt="Image 2" src="https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/trajectory_after_mpc-2d.png" width="45%" />
</p>

**ILC-QP (Example)**
<!-- ![ilc-qp](https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/trajectory_after_ilc-qp.png) -->
<p align="center">
  <img alt="Image 1" src="https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/trajectory_after_ilc-qp.png" width="45%" />
  <img alt="Image 2" src="https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/figs/error_convergence_ilc-qp.png" width="45%" />
</p>

## Run Locally

Clone the project

```bash
  git clone https://github.com/lihanlian/trajectory-tracking-ilc
```

Go to the project directory
 - run _get_reference_trajectory.m_ to generate mat file that stores vector of reference trajectories (_reference_trajectory.mat_)
 - run _get_initial_control_input.m_ to generate mat file that stores vector of initial control input solved by direct collocation, which will then be used for ILC iteration. (_initial_control_input.mat_)
 - run _get_ilc_matrix.m_ to generate mat file that stores necessary matrices within ILC iteration loop. (_ilc_matrix_nominal.mat_ and _ilc_matrix_actual.mat_)
 - run _controller_mpc.m_ to check the tracking performance using MPC controller.
 - run _controller_ilc_*._m_ to check the tracking performance using different ILC algorithms.


## Acknowledgements
The author would like to appreciate the course instructor Professor Uduak Inyang-Udoh for his help throughout the semester and discussion during office hours. The author would also like to thank Professor Zachary Manchester and Professor Pranav Bhounsule for the open course CMU 16-745 and open course on Robotics. Below are some good resources might be helpful on related topics.
 - [Lecture on Iterative Learning Control - Prof. Zachary Manchester](https://www.youtube.com/watch?v=JXZbrzJiUo4&list=PLZnJoM76RM6KugDT9sw5zhAmqKnGeoLRa&index=29)
 - [Lecture on Trajectory Optimization (Direct Collocation and Shooting Method) - Prof. Pranav Bhounsule](https://github.com/matiassingers/awesome-readme)
 - [MATLAB Example for Iterative Learning Control](https://www.mathworks.com/help/slcontrol/ug/model-free-iterative-learning-control-of-siso-system.html)
 - [Blog about Direct Collocation](https://sam.pfrommer.us/tutorial-direct-collocation-trajectory-optimization-with-matlab)
- [YouTube video about Deep Deterministic Policy Gradient (DDPG)](https://www.youtube.com/watch?v=oydExwuuUCw&t=282s&ab_channel=AylwinWei)


## License

[MIT](https://github.com/lihanlian/trajectory-tracking-ilc/blob/main/LICENSE)