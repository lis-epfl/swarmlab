# SwarmLab
<p align="center"><img src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/SwarmLab.svg" alt="Logo" height="200"></p>
 
<br>
<br>

SwarmLab is a drone swarm simulator written in Matlab. It aims at:
1. simulating *single drones*;
2. simulating *swarms of drones*;

This software is designed with _versatility_ and _scalability_ in mind. It allows for fast programming and easy incorporation of various modules designed for drone and drone swarm simulations. It  also decreases the coding effort by offering built-in and ready-to-use functionalities. These features make this package suitable to different applications in the area of drones and swarm robotics, including fast algorithm  development, training, education, and the automatic collection of simulated data.

<p align="center">
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/Scheme.jpg" alt="scheme of architecture"/>
</p>

## Requirements
Only [Matlab installation](https://www.mathworks.com/support/install-matlab.html?q=&page=1) is required.

## 1. Single-drone simulations
Single-drone simulations exploit the architecture proposed by Professors Beard and McLain in [Small unmanned aircraft: theory and practice](https://press.princeton.edu/books/hardcover/9780691149219/small-unmanned-aircraft) and illustrated below. The focus of these simulations is realism. Their code is available [here](https://github.com/randybeard/mavsim_template_files).
Two drone-types are supported: quadcopter and fixed-wing drones.

<p align="center">
<img width=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/drone_architecture.png" alt="drone simulation architecture"/>
</p>

### Graphics
The graphical tools supported for single drone simulations are:
* 3D drone viewer
* state variable plotter

<p align="center">
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/drone_viewer.png" alt="drone viewer"/>
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/drone_state_plotter.png" alt="drone state plotter"/>
</p>

### Examples
The following examples are provided:
1. controller
2. path follower
3. path manager
4. path planner

<p align="center">
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/path_manager.png" alt="path manager example"/>
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/path_planner.png" alt="path planner example"/>
</p>

### GUI

For ease of use, single-drone simulations can also be run from a dedicated GUI, that allows to change a selection of parameters, e.g. drone type, simulation type, debugging plots, simulation time.

<p align="center">
<img width=400 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/GUI_drone.png" alt="GUI drone"/>
</p>

## 2. Drone-swarm simulations
Drone swarm simulations exploit either the [Olfati-Saber](https://www.sciencedirect.com/science/article/pii/S1474667015386651) or the [Vicsek (Vásárhelyi’s version)](https://robotics.sciencemag.org/content/3/20/eaat3536.short) algorithms. The focus of these simulations is the behaviour of the group of drones, as a result of interactions among individuals. In this case, quadcopter and point-mass (featuring no vehicle dynamics) are supported. The latter can be used when computational time requirements prevail over simulation realism, or when the interest relies in the pure collective behavior, independently on the agents' dyanamics.

### Graphics
The graphical tools supported for drone swarm simulations are:
* run-time 3D swarm viewer
* run-time state variable plotter
* offline 3D swarm viewer with wakes
* offline state variable plotter
* offline performance analyser

<p align="center">
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/swarm_traj.png" alt="swarm trajectories"/>
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/swarm_dist.png" alt="swarm distances"/>
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/swarm_speed.png" alt="swarm speed"/>
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/swarm_performance.png" alt="swarm performances"/>
</p>

### Examples
The following examples are provided:
1. Olfati Saber's swarming algorithm
2. Vasarhelyi's version swarming algorithm

The following illustrations represent:
* a swarm of 5 drones with quadcopter dynamics, 
* a swarm of 25 drones with point mass dynamics
before and after convergence to the equilibrium configuration.

<p align="center">
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/point_mass_swarm.png" alt="point mass swarm"/>
</p>

<p align="center">
<img height=300 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/quad_swarm.png" alt="quadcopter swarm"/>
</p>

### GUI
Also drone-swarm simulations can be run from a dedicated GUI. In this case, parameters that can be set are: drone types, simulation time, debug plotting, map plotting, number of agents of the swarm, swarming algorithm, preferred inter-agent distance, preferred orientation, preferred speed.
The GUI allows for run-time changes of some parameters, such as the swarm direction.


<p align="center">
<img width=400 src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/GUI_swarm.png" alt="GUI swarm"/>
</p>

## Basic usage


To start off with your first drone swarm simulation, open the project folder <i>swarmlab</i> in Matlab (see picture aside). Then, add all folders and subfolders to the current path (select the swarmlab folder → right-click → Add to Path → Selected Folders and Subfolders). 
You have two ways of running your simulations, either via the provided GUIs or by calling an example script.
 * For the GUI, type `GUI_drone` or `GUI_swarm` on the Matlab command view, select the parameters you want and slide to `On` the _Start Simulation_ button.
 * For running an example, type `example_vasarhelyi`
For editing the example scripts, go to examples → examples_swarm, and open the <i>example_vasarhelyi.m</i> script.
</p>

Finally, simply run the script and pay attention to the prompted instructions!

<p align="center">
<img align="center" width="300" height="450" src="https://github.com/lis-epfl/swarmlab/blob/master/docs/images/project_folder.png">
</p>

## Acknowledgements

Thanks to Victor Delafontaine, Andrea Giordano, and Anthony De Bortoli for their valuable contribution. Thanks to Dario Floreano and Fabrizio Schiano for their wise advice.

## Reference

* [Swarmlab: a Drone Swarm Simulator](https://ieeexplore.ieee.org/document/9340854)

Please, cite us with:
```
@INPROCEEDINGS{9340854,
  author={Soria, Enrica and Schiano, Fabrizio and Floreano, Dario},
  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={SwarmLab: a Matlab Drone Swarm Simulator}, 
  year={2020},
  volume={},
  number={},
  pages={8005-8011},
  doi={10.1109/IROS45743.2020.9340854}}
```

## Licensing
This software is provided under MIT License (MIT). 
Copyright (c) 2020 Enrica Soria.

#### _Enjoy!_ :+1:
