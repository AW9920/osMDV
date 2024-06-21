# Open-Source Desktop Teleoperated Surgical Training System
![image](https://github.com/AW9920/osMDV/assets/76056168/466618e0-05de-4815-afaa-288cf0a57a3f)


Welcome to the repository for the **Open-Source Desktop Teleoperated Surgical Training System**. This project provides cost-efficient and accessible resources for a surgical training system.


## Features

- **Master-Tool-Manipulator (MTM)**: A serial link manipulator capable of measuring the operator’s motion in 7 degrees of freedom (DoF), including 3 DoF for hand translation, 3 DoF for wrist rotation, and 1 DoF for grasping motion.
- **Patient-Side-Manipulator (PSM)**: Based on the da Vinci Classic system, this manipulator features a double parallelogram mechanism and is capable of precise motion replication.
- **Kinematic Models**: Detailed kinematic models for both MTM and PSM using the Denavit-Hartenberg (DH) method.
- **Gravity-Compensation-System (GCS)**: Simulated spatial positions of the center of gravity for counterbalancing mechanisms, designed using a least-square fitting algorithm.

## Repository Contents

- **CAD Drawings**: Detailed CAD drawings for the MTM and PSM, including all necessary components and assembly instructions.
- **Source Code**: Control algorithms, kinematic models, and simulation scripts necessary for operating the MTM and PSM.
- **Calculations**: Documentation of the kinematic and dynamic calculations used in the system design.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.
