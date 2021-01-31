# SLAM
## Simultaneous Localization and Mapping (SLAM)

This repo documents the development of a
two dimensional robot localization and mapping
algorithm based on a purpose-built Sequential
Monte Carlo algorithm.
The model processes discrete wheel rotation data
collected from self-contained wheel encoders, as
well as light radar (LIDAR) detection data over a
270 degree horizontal visual field. In addition, a robot
inertial measurement unit (IMU) provides linear
and angular acceleration data.
Using these inputs, the model iteratively produces
a correlation to observed map landmarks (i.e.:
prior LIDAR returns from objects in the
environment) to predict the most likely location
of the robot. According to this best-estimate
location, landmark location confidences are
incrementally updated using Bayesian inference.
