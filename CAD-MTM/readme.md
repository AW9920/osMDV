# CAD of Open-Source Desktop Teleoperated Surgical Training System

This folder provides mechanical assembly and CAD drawings.

## Overview

This section of the repository focuses on the mechanical aspects of the teleoperated surgical training system, including the Master-Tool-Manipulator (MTM) and Patient-Side-Manipulator (PSM). These components are designed to be affordable and easy to assemble, reducing the entry barrier for research and educational purposes.

## Mechanical Assemblies

### Master-Tool-Manipulator (MTM)

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

### Patient-Side-Manipulator (PSM)

The PSM is based on the da Vinci Classic system and features a double parallelogram mechanism for precise motion replication.

**Components:**

- **Frame**: The main supporting structure for the PSM.
- **Joints**: Seven joints, actuated by DC motors (273752, MAXON).
- **Servo Motors**: Four servo motors (HS-65HB+, Hitec) for wrist movements.
- **Optical Encoders**: Encoders (HEDS-5540#A11, Broadcom) for angular position measurement.
- **Surgical Instrument Mount**: A sled to accommodate surgical instruments.

**DPM Base**
![ForkMecha](https://github.com/AW9920/osMDV/assets/61975888/e7f96d02-b445-4a05-ae61-6f713d43c98f)
**DPM**
![DPM_v2](https://github.com/AW9920/osMDV/assets/61975888/fa76ceb9-c7b3-4b58-b465-8e763ed9f82a)
**Base Shaft**
![AX0_new](https://github.com/AW9920/osMDV/assets/61975888/51080bb7-d4ce-406e-9bf8-bc24dfeb413c)
**Head Assembly**
![Guidance](https://github.com/AW9920/osMDV/assets/61975888/49ca5d13-6fcb-4db0-88bb-3e79a4705c55)
**Tool Carriage**
![Carriage_v2](https://github.com/AW9920/osMDV/assets/61975888/067e491d-3ef2-469a-8119-9396151d73dc)


