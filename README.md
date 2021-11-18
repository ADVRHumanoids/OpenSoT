OpenSoT MultiDoF (v3.0)  ![Travis badge](https://app.travis-ci.com/ADVRHumanoids/OpenSoT.svg?token=iQZzUD6rXcPydQAabf7T&branch=devel)
============
An Open Source Task Solving library with Constraints

Change Log (short):
- Linear Tasks
- iHQP/LP solver
- Inverse Dynamics & Operational Space
- Contact Force Optimization
- Added eiQuadProg
- Added GLPK

Available Solvers:
------------------
- iHQP/LP: implemented using [qpOASES](https://projects.coin-or.org/qpOASES), [OSQP](http://osqp.readthedocs.io/en/latest/), [eiQuadProg](https://www.cs.cmu.edu/~bstephe1/eiquadprog.hpp), [GLPK](https://www.gnu.org/software/glpk/) 
- eHQP: implemented using Eigen-based Damped Pseudo Inverse
- nHQP: Null-Space Hierarchical QP, implemented using Eigen SVD

The default iHQP solver is based on qpOASES and eiQuadProg. 

Introduction
------------
OpenSoT is a library dedicated to hierarchical whole-body control of robots subject to constraints such as joint limits, joint velocities, cartesian constraints... The main idea behind OpenSoT is to decouple Task and Constraints description from the Solver used to compute the robot commands and the Type of Control available on the robot to perform them.

OpenSoT was initially developed under the EU Project WALK-MAN (http://www.walk-man.eu/).
At the moment the project is developed under the EU Projects CogIMon (https://cogimon.eu/) and CENTAURO (http://www.centauro-project.eu/). 

Its homepage resides in http://github.com/robotology-playground/OpenSoT
You can also find a wiki there.

An online version of this documentation can be obtained in http://opensot.github.io

Some videos from OpenSoT YouTube channel (https://www.youtube.com/channel/UCkkZXunCN6eJwww1CeK7DrA):

<a href="https://www.youtube.com/watch?v=Q1u2vZ0dhh0
" target="_blank"><img src="http://img.youtube.com/vi/Q1u2vZ0dhh0/0.jpg" 
alt="Self Collision Avoidance" width="480" height="360" border="10" /><br>Self Collision Avoidance</a>

<a href="https://www.youtube.com/watch?v=-n3jxAZaK5Q
" target="_blank"><img src="http://img.youtube.com/vi/-n3jxAZaK5Q/0.jpg" 
alt="WALK-MAN Whole-Body, floating-base, walking" width="480" height="360" border="10" /><br>WALK-MAN Whole-Body, floating-base, walking</a>

Add-ons
------------
- [CartesI/O](https://github.com/ADVRHumanoids/CartesianInterface)
- [Visual Servoing](https://github.com/EnricoMingo/opensot_visual_servoing), developed in collaboration with IDSIA - USI/SUPSI (Antonio Paolillo)

Developers:
-----------
Enrico Mingo Hoffman  
Arturo Laurenzi  
Matteo Parigi Polverini  
Franesco Ruscelli

Previous Developers:
--------------------
Alessio Rocchi  
Cheng Fang


How to cite this work:
======================
Please support ```OpenSoT``` development by referencing it in your works/publications/projects with:

```
@inproceedings{OpenSot17,
  title = {Robot Control for Dummies: Insights and Examples using OpenSoT},
  author = {Mingo Hoffman, Enrico and Rocchi, Alessio and Laurenzi, Arturo and Tsagarakis, Nikos G.},
  pages     = {736-741},
  booktitle = {17th {IEEE-RAS} International Conference on Humanoid Robots, Humanoids},
  year = {2017}
}
```

```
@inproceedings{AlessioEnrico2015,
  title={OpenSoT: a Whole-Body Control Library for the Compliant Humanoid Robot COMAN},
  author={Rocchi, Alessio and Hoffman, Enrico Mingo and Caldwell, Darwin G. and Tsagarakis, Nikos G.},
  booktitle={Robotics and Automation (ICRA), 2015 IEEE International Conference on},
  pages={1093--1099},
  year={2015},
  organization={IEEE}
}
```
