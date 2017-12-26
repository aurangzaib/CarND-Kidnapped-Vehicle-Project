# Kidnapped Vehicle Project using Particle Filter (PF)

| **Source Code**  | [https://github.com/aurangzaib/CarND-Kidnapped-Vehicle-Project](https://github.com/aurangzaib/CarND-Kidnapped-Vehicle-Project)  |
|:-----------|:-------------|
| **Overview**  | `README.md`  |
| **Setup**  | `SETUP.md`  |
|**PF Implementation**| `src/particle_filter.cpp`|
| **How to run**  | `mkdir build && cd build` | 
| |`cmake .. && make`     	|
| |`./particle_filter`     		|

## Introduction:

A robot is transported to a new location where current location of the robot is unknown.

A two dimensional Particle Filter is implemented to localize the robot by providing the best estimate of the current location of the robot.

##### Input to Particle Filter:

- A map of robot's location with landmarks
- A noisy GPS estimate (x, y, heading) of robot's initial location
- Lidar sensor noisy observations (x, y)
- Noiseless control data (velocity and turn rate)

##### Output of Particle Filter:

- Best estimate of robot's location (x, y, orientation)
- Lidar sensor observations (x, y) which have best associations (nearest neighbours) with map landmarks

##### The steps of the project are as following:

- Read map landmarks (x, y) and noisy GPS data (x, y, theta)
- Initialize Particle Filter.
- Read previous velocity (v) and turn rate (yaw_rate)
- Predict next state.
- Read Lidar sensor noisy observation (x, y) and transform it from Vehicle Coordinate System (VCS) to Map Coordinate System (MCS)
- Update each particle's importance weights
- Resample particles with respect to their importance weights

## Explanation of the code:

The implementation of the project is divided into 2 files:

`main.cpp`

`particle_filter.cpp`


Following table summarizes the purpose of each file:

| File | Explanation |
|:-----------|:-------------|
|**main.cpp**| |
|				| Define uncertainties of GPS and map landmarks measurements |
| 				| Read map landmarks (x, y) and noisy GPS data (x, y, theta) | 
|				| Call `init` method of class `ParticleFilter` |
|				| Read previous velocity (`v`) and turn rate (`yaw_rate`) |
|				| Call `prediction` method of class `ParticleFilter` |
|				| Read noisy Laser sensor observation data |
|				| Call `update_weights` method of class `ParticleFilter` |
|				| Call `resample` method of class `ParticleFilter` |
|				| Call `get_sense_x` and `get_sense_y` method of class `ParticleFilter` to get best location estimate of robot|
|				| Call `get_associations` method of class `ParticleFilter` to get associations with map landmarks|
|**particle_filter.cpp**| |
|`init` | |
| 				| Define `num_particles` |
| 				| Set each particle's position and orientation |
| 				| Set each particle's weight to 1 |
|`prediction` | |
| 				| Update each particle's position (`x, y`) and orientation (`theta`) from given velocity and turn rate (`velocity, yaw_rate`) |
|`update_weights` | |
| 				| Transform Laser sensor observation from VCS to MCS | 
| 				| Find nearest landmark to each observation using distance formula |
|				| Calculate normalized probability for each observation using multi-variate Gaussian distribution |
|				| Update each particle's weight by combining normalized probabilities | 
|`resample`	|	|
|				| Use `Resample Wheel` algorithm to reorder particles with respect to their importance weights |


## Results:

The project is successfully passed:

![Results](result-pf.png)

The video (GIF) below shows results of object localization using Particle Filter:

![Results](result-pf.gif)
