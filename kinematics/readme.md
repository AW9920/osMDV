# Kinematic Model - MTM

![MTM_Kinematics_v2](https://github.com/AW9920/osMDV/assets/61975888/41506908-b114-4d83-a015-4890bf454e18)

## **DH-Parameters of the MTM**

| i | theta_i / rad | d_i / mm | a_i / mm | alpha_i / rad |
|---|---------------|----------|----------|---------------|
| 1 | q_1           | 0        | l_sho    | pi/2          |
| 2 | q_2           | 0        | l_arm    | -pi/2         |
| 3 | q_3 - pi/2    | -l_ball  | l_arm    | -pi/2         |
| 4 | q_4 - pi/2    | 0        | 0        | -pi/2         |
| 5 | q_5 - pi/2    | 0        | 0        | -pi/2         |
| 6 | q_6 + pi/2    | -l_IMU   | 0        | -pi/2         |

With geometric parameters being:
l_arm = 217.50 mm
l_ball = 8.70 mm
l_IMU = 37.40 mm

# Kinematic Model - PSM

![PSM_Kinematics_v2](https://github.com/AW9920/osMDV/assets/61975888/8c46b517-5b5f-40d9-ad9e-4cad083b5da8)

##  **DH-Parameters of the PSM**

| i | theta_i / rad |  d_i / mm   | a_i / mm | alpha_i / rad |
|---|---------------|-------------|----------|---------------|
| 1 | q_1           | 0           | 0        | -pi/2         |
| 2 | q_2 - pi/2    | 0           | 0        | pi/2          |
| 3 | 0             | q_3 - l_RCM | 0        | 0             |
|   | 0             | l_tool      | 0        | 0             |
| 4 | q_4           | 0           | 0        | -pi/2         |
| 5 | q_5 - pi/2    | 0           | l_P2Y    | -pi/2         |
| 6 | q_6           | 0           | l_Y2T    | 0             |

With geometric parameters being:
l_RCM   = 456.00 mm
l_P2Y   = 9.00 mm
l_tool  = 432.50 mm
l_Y2T   = 10.10 mm

# Inverse Kinematics Wrist Rotation

The analytical form of 3R6 is derived from the DH parameters of the PSM, whereas the elements of the matrix depicted in
the second equation poses numerical values resulting from the transformation of IMU values in Rotation Matrix form (0R6).

### Matrices

$$
^3R_6 = \begin{bmatrix}
-c_4 c_6 - s_4 s_5 s_6 & -s_4 c_5 & c_4 s_6 + s_4 s_5 c_6 \\
s_4 c_6 + c_4 s_5 s_6 & c_4 c_5 & s_4 s_6 + c_4 s_5 c_6 \\
-c_5 s_6 & s_5 & -c_5 c_6
\end{bmatrix}
$$

$$
^0R_3^{-1} \cdot ^0R_6 = \begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix}
$$



Based on the detailed notation, it is determined that joint values q4, q5, and q6 can be estimated according to the equations listed below.

$$
\[ q_4 = \arctan2\left(-\frac{r_{12}}{c_5}, \frac{r_{22}}{c_5}\right) \]

\[ q_5 = \arctan2\left(r_{32}, \pm\sqrt{(r_{12})^2 + (r_{22})^2}\right) \]

\[ q_6 = \arctan2\left(-\frac{r_{31}}{c_5}, -\frac{r_{33}}{c_5}\right) \]
$$

# Interpretation of Disk Rotation

The discs of the surgical instrument must be actuated properly to authentically recreate the previously calculated joint values

##  Correlation Matrix
The transmission rate between the disc number and corresponding joint values are estimated and listed below.

