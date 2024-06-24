# Electronics MTM

## General Description of the Electrical Setup MTM
The robotic arm system incorporates sensors and controllers to detect movements and control various joints. Each side of the arm is equipped with sensors for shoulder pitch, shoulder yaw, elbow pitch, wrist, and grip, all centrally controlled by an Arduino Mega.

### Shoulder Pitch Subsystem:
- **Magnetic Encoder (AEAT-6012-A06)**: Provides precise feedback on the shoulder pitch position.

### Shoulder Yaw Subsystem:
- **Magnetic Encoder (AEAT-6012-A06)**: Measures the shoulder yaw angle accurately.

### Elbow Pitch Subsystem:
- **Magnetic Encoder (AEAT-6012-A06)**: Monitors the elbow pitch movement.

### Wrist Subsystem:
- **MPU 6050**: Tracks the wrist orientation and movement. The MPU measures 3 degrees of freedom.

### Grip Subsystem:
- **Hall Sensor**: Detects the grip force or position. The location of the measurement is the grip between index finger and thumb.

### Controller
- **Arduino Mega**: The central controller orchestrates the operations of all subsystems. It processes input signals from the encoders, MPU 6050 sensors, and Hall sensors and sends appropriate control signals to achieve coordinated and precise movements of the robotic arm.

## Electrical Schematic MTM
![image](https://github.com/AW9920/osMDV/assets/76056168/c7714793-d4a3-4a42-a1a7-49d75c5d7415)

![image](https://github.com/AW9920/osMDV/assets/76056168/8caa71ce-837c-4679-90d1-c0537a82bebb)


# Electrical Component PSM

## General Description of the Electrical Setup
The MTM detects movements in 7 dimensions of freedom. Consequently, the PSM requires 7 motors to replicate these movements. Servo motors, which have integrated position control, are utilized for wrist movements. For the outer yaw, pitch, and insertion, DC motors are employed, and these motors need additional encoders for position control. To calibrate the encoders, the PSM uses limit switches to detect the end positions.

The electrical system of the project is composed of three main subsystems: outer pitch, outer yaw, and insertion. Each subsystem is managed by a central controller, the Teensy 4.1, which interfaces with various sensors, motors, and motor drivers to ensure precise and coordinated movement.

### Outer Pitch Subsystem
- **Limit Switch**: An Omron D2F-L-A1 limit switch is used to detect the end-of-travel positions for safety and calibration purposes.
- **Encoder**: A Broadcom HEDS-5540 encoder provides feedback on the rotational position and speed of the DC motor.
- **DC Motor**: A Maxon 273752 15V DC motor is responsible for generating the necessary torque and motion.
- **Motor Driver**: An MD13S Cytron 13 Amp motor driver controls the power supplied to the DC motor based on commands from the controller.

### Outer Yaw Subsystem
- **Limit Switch**: Another Omron D2F-L-A1 limit switch is employed to detect the limits of travel.
- **Encoder**: The Broadcom HEDS-5540 encoder offers precise feedback on the motor's position and velocity.
- **DC Motor**: A similar Maxon 273752 15V DC motor is used for the yaw movement.
- **Motor Driver**: The MD13S Cytron 13 Amp motor driver modulates the power delivered to the DC motor.

### Insertion Subsystem
- **Limit Switch**: The Omron D2F-L-A1 limit switch is utilized for detecting the extreme positions.
- **Encoder**: The Broadcom HEDS-5540 encoder measures the rotational position for precise control.
- **Servo Motor**: A servo motor (HV7346MG) with position control removed is used to drive the insertion mechanism.
- **Motor Driver**: The same MD13S Cytron 13 Amp motor driver is used to control the servo motor.

### Wrist Subsystem
The wrist subsystem comprises multiple servo motors (MG966r) for fine-tuned multi-axis control. These servos are directly controlled by the Teensy 4.1 controller to achieve the desired movements.

### Controller
- **Teensy 4.1**: The central controller that orchestrates the operations of all subsystems. It processes input signals from encoders and limit switches and sends appropriate control signals to the motor drivers and servo motors.



![image](https://github.com/AW9920/osMDV/assets/76056168/6fb0c4c8-1e95-4db8-a5aa-cfffba504cfd)

## Electrical Schematic PSM

![image](https://github.com/AW9920/osMDV/assets/76056168/54f3b947-939d-432c-a1ec-a0965999e298)

![image](https://github.com/AW9920/osMDV/assets/76056168/8cd2c804-6cf9-4c23-aa64-0961a717756b)

![image](https://github.com/AW9920/osMDV/assets/76056168/97f33d15-e122-4ada-9089-cb1507c8201d)
