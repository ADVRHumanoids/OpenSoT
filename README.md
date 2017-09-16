OpenSoT Lite (v2.0)
============
New version of OpenSoT based mainly on Eigen3.0.
Most of the code has been ported to Eigen to be real time safe. The Lite version represent a working progress in which we want to try to achieve real time performances where is possible. 

An Open Source Task Solving library with Constraints

Introduction
------------
OpenSoT is a library dedicated to hierarchical whole-body control of robots subject to constraints such as joint limits, joint velocities, cartesian constraints... The main idea behind OpenSoT is to decouple Task and Constraints description from the Solver used to compute the robot commands and the Type of Control available on the robot to perform them.

OpenSoT is developed under the European Project WALK-MAN: http://www.walk-man.eu/.

Its homepage resides in http://github.com/robotology-playground/OpenSoT
You can also find a wiki there.

An online version of this documentation can be obtained in http://opensot.github.io

Some videos from OpenSoT channel in YouTube:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Avs6pqUCqVo
" target="_blank"><img src="http://img.youtube.com/vi/Avs6pqUCqVo/0.jpg" 
alt="OpenSoT + PI" width="480" height="360" border="10" /><br>OpenSoT + PI</a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=cBggRy7WPUE&list=UUkkZXunCN6eJwww1CeK7DrA
" target="_blank"><img src="http://img.youtube.com/vi/cBggRy7WPUE/0.jpg" 
alt="Minimum Effort + Drawing Task" width="480" height="360" border="10" /><br>Minimum Effort + Drawing Task</a>

<a href="https://www.youtube.com/watch?v=aYwSLnBZtuA
" target="_blank"><img src="http://img.youtube.com/vi/aYwSLnBZtuA/0.jpg" 
alt="CoM Vs Waist Walking with the SoT" width="480" height="360" border="10" /><br>CoM Vs Waist Walking with the SoT</a>

Installation
------------

To Install OpenSoT, the recommended way is to use the OpenSoT-superbuild https://github.com/EnricoMingo/OpenSoT-superbuild

How to cite this work:
======================
Please support ```OpenSoT``` development by referencing it in your works/publications/projects with:
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

```
@inproceedings{Mingo16,
  title={Robot Dynamics Constraint for Inverse Kinematics},
  author={Mingo Hoffman, Enrico and Rocchi, Alessio and Tsagarakis, Nikos G. and Caldwell, Darwin G.},
  booktitle={International Conference on Advances in Robot Kinematics, ARK 2016},
  pages={280--286},
  year={2016},
  organization={IFToMM}
}
```

```
@inproceedings{Fang15,
  added-at = {2016-01-06T00:00:00.000+0100},
  author = {Fang, Cheng and Rocchi, Alessio and Hoffman, Enrico Mingo and Tsagarakis, Nikos G. and Caldwell, Darwin G.},
  biburl = {http://www.bibsonomy.org/bibtex/2fed92aad7fab0089cd092a76f2d6e819/dblp},
  booktitle = {Humanoids},
  ee = {http://dx.doi.org/10.1109/HUMANOIDS.2015.7363500},
  interhash = {a4dd115293f874acd3b67c9781b45a40},
  intrahash = {fed92aad7fab0089cd092a76f2d6e819},
  isbn = {978-1-4799-6885-5},
  keywords = {dblp},
  pages = {1060-1066},
  publisher = {IEEE},
  timestamp = {2016-01-07T11:44:55.000+0100},
  title = {Efficient self-collision avoidance based on focus of interest for humanoid robots.},
  year = 2015
}
```

