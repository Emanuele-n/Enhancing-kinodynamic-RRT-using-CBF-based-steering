# Enhancing-kinodynamic-RRT-using-CBF-based-steering
Final Project for the course in Autonomous and Mobile Robotics A.Y. 2020/2021.

## Project Description 
RRT-based planners, at each iteration, generate a subpath in the configuration space, and extend the tree
only if no collision occurs along it. The aim of this project is to implement a RRT-like planner that uses an
optimization-based steering method which incorporates an explicit collision avoidance constraint
formulated via Control Barrier Functions (CBFs), a tool that has been recently introduced for formulating
safety-related constraints. In principle, this allows to guide the generation of subpaths in the free
configuration space, thus reducing the number of failed extension attempts and eliminating the need of
collision checks. The planner has been be tested through V-REP/CoppeliaSim simulations in different
scenarios using a nonholonomic system (e.g., a unicycle) and a critical comparison with the basic RRT
planner has been carried out.

The main file is [v_repExtAMRProject](v_repExtAMRProject.cpp)

## Documentation
A detailed [report](AMR_report.pdf) of the project<br />
The [presentation](AMR_FP6_presentation.pptx) we gave

## Resources used
* [CoppeliaSim](https://www.coppeliarobotics.com/)<br />
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)<br />
* [qpOASES](https://www.coin-or.org/qpOASES/doc/3.2/doxygen/index.html)

## Authors
* G. Giunta <br />
* E. Nicotra <br />
* A. Paggetti
