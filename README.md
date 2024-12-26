# Convex MPC on pointfoot robot

## Overview  
This repository contains the code for the convex MPC on the pointfoot robot. (Still in progress)

## Installation
1. Clone the repository
```bash
git clone
```

2. Install the dependencies
```bash
pip install -r requirements.txt
```

## Usage
1. Run the convex MPC on the pointfoot robot in Mujoco
```bash
python script/pointfoot_simulator.py  
python script/mpc_controller.py  
```

2. Deploy the convex MPC on the pointfoot robot in real world (TODO)

## Demonstration
1. Convex MPC on the pointfoot robot in Mujoco

2. Convex MPC on the pointfoot robot in real world (TODO)

## Acknowledgements
- J. Di Carlo, P. M. Wensing, B. Katz, G. Bledt and S. Kim, "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control," 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp. 1-9, doi: 10.1109/IROS.2018.8594448. keywords: {Robot kinematics;Legged locomotion;Dynamics;Predictive control;Convex functions;Predictive models}

- [pympc-quadruped] (https://github.com/yinghansun/pympc-quadruped)

- [VF-TSC] (https://github.com/clearlab-sustech/VF-TSC)
