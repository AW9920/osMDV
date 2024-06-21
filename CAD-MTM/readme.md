# CAD of the patient side manipulator

This folder provides mechanical assembly and CAD drawings.

## Overview

This section of the repository focuses on the mechanical aspects of the teleoperated surgical training system, including the Master-Tool-Manipulator (MTM). These components are designed to be affordable and easy to assemble, reducing the entry barrier for research and educational purposes.

## Mechanical Assemblies

The MTM is a serial link manipulator designed to measure the operator's motion in 7 degrees of freedom (DoF), including:

- 3 DoF for hand translation
- 3 DoF for wrist rotation
- 1 DoF for grasping motion

**Components:**

- **Frame**: The main structure supporting the MTM.
- **Joints**: Seven joints allowing for the specified degrees of freedom.
- **Encoders**: Three absolute magnetic encoders (AEAT-6012-A06, Broadcom) for measuring hand translation.
- **IMU**: A 6-axis inertial measurement unit encased in a wrist control unit (WCU) for wrist rotation estimation.
- **Hall Sensor**: A sensor (SS495A1, Honeywell) for grasping motion detection.

![Revised_MTM_v2](https://github.com/AW9920/osMDV/assets/61975888/232f6bfd-2131-440a-a03f-9592015d7144)




