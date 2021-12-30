## magnetometer calibrate

This repository is the c++ code to perform magnetimeter calibrattion using Eigen. We reference the matlab code from <https://github.com/Razor-AHRS/razor-9dof-ahrs/tree/master/Matlab/magnetometer_calibration>. The calibration result from this c++ code is completely equal to the result from matlab code.



You can search further information about the magnetometer calibration by the keyword `hard iron` and `soft iron`. In this code we use the `ellipsoid fit` method to correct `soft iron` effects. This method assumes that the measurements of the magnetometer will have an ellipsoid like distribution, and with a simple transformation, it is transformed into an sphere.

Note that we can describe and ellipse with the following equiations using `A,B,C,D,E,G,H,I` parameters.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?Ax^2&space;&plus;&space;By^2&space;&plus;&space;Cz^2&space;&plus;&space;2Dxy&space;&plus;&space;2Exz&space;&plus;&space;2Fyz&space;&plus;&space;2Gx&space;&plus;&space;2Hy&space;&plus;&space;2Iz">
</p>

With the following expression we can do this transformation. Note that `c` is the center of the transformation and `exy` is the `xy` element of the transformation matrix. The code available in this repository, will give you this two matrices. One 3x3 and one 3x1. Once you have them, you are ready to calibrate your magnetometer measurements.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?\begin{pmatrix}&space;mag_x&space;\\&space;max_y&space;\\&space;max_z&space;\\&space;\end{pmatrix}&space;=&space;\begin{pmatrix}&space;e_{11}&space;&&space;e_{12}&space;&&space;e_{13}\\&space;e_{21}&space;&&space;e_{22}&space;&&space;e_{23}\\&space;e_{31}&space;&&space;e_{32}&space;&&space;e_{33}\\&space;\end{pmatrix}\begin{pmatrix}&space;mag_x&space;-&space;c_x&space;\\&space;max_y&space;-&space;c_y\\&space;mag_z&space;-&space;c_z\\&space;\end{pmatrix}">
</p>

### data

we provide magnetometer data in the folder data. The txt file contains four columns which is :

```c++
[time, mag_x, mag_y, mag_z]
```



### Prerequisites

- Eigen
