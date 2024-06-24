# General Functionality
The PSM code is designed to accurately replicate the movements detected by the MTM. It does so by receiving movement data from the MTM, parsing and processing this data, and then using it to control the PSM's motors and servos. The system uses PID control to ensure precise movements and implements various safety and efficiency measures such as ramp-up routines, collision detection, and low-pass filtering.

## MT_PSM.ino
This is the main control file for the PSM system. Key functionalities include:
- Initializing the PSM components and setting up initial configurations.
- Reading data from the MTM and converting it into commands for the PSM.
- Handling the overall workflow (see the following picture) of the PSM to ensure it accurately replicates MTM movements.

  ![image](https://github.com/AW9920/osMDV/assets/76056168/9bf5e672-8922-49c9-84f3-e03dfdd994ca)


## parseData.ino
This file contains the logic to parse incoming data strings from the MTM. Key functionalities include:
- Tokenizing received data strings.
- Converting tokenized strings into numerical values.
- Storing parsed data into appropriate variables for further processing.

## PIDupdate.ino
This file implements PID (Proportional-Integral-Derivative) control for the motors. Key functionalities include:
- Calculating PID values to control motor speed and position.
- Adjusting motor commands based on the PID calculations to ensure precise movements.

## PosSampler.ino
This file handles the sampling of position data. Key functionalities include:
- Reading position data from sensors.
- Storing sampled data for use in control algorithms.
- Ensuring that position data is accurately and consistently captured.

## RampUp_Homing.ino
This file implements ramp-up and homing routines for the PSM. Key functionalities include:
- Performing homing sequences to establish a known starting position for the PSM.

## recvWithStartEndMarkers.ino
This file handles the reception of data with defined start and end markers. Key functionalities include:
- Detecting start and end markers in incoming serial data.
- Extracting and storing data between the markers.
- Setting flags to indicate new data availability for processing.

## setPwmFrequency.ino
This file sets the PWM (Pulse Width Modulation) frequency for the motors. Key functionalities include:
- Configuring PWM settings for motor control.
- Ensuring that motors operate at the correct frequency for smooth and precise movements.

## Catch_Tool.ino
This file contains routines to catch and manipulate tools. Key functionalities include:
- Controlling servos to grip the tool securely.
- Returning to the initial position after catching the tool.

## CheckCol.ino
This file checks for collision conditions during operation. Key functionalities include:
- Monitoring limit switches to detect collisions.
- Stopping motors if a collision is detected to prevent damage.
- Ensuring safe operation by avoiding excessive forces.

## LPFilter_DT.ino
This file implements a low-pass filter for delta time signals. Key functionalities include:
- Filtering delta time measurements to remove noise.
- Ensuring stable and accurate time measurements for control algorithms.

# Referencing
Since incremental encoders are utilized for joint axis 1, 2, and 3, the system must conduct a reference drive before it is ready for operation. This is necessary since the encoders do not possess an absolute point of reference, which causes a loss of position whenever the program execution is terminated. Therefore, a state machine is implemented to guide each joint towards a limit switch positioned at a specific angle. Since the encoders of the remaining joints are able to measure the absolute position at any time, they are excluded from the reference drive procedure. The protocol of the state machine is visualized by the flow chart in Figure.

![image](https://github.com/AW9920/osMDV/assets/76056168/a42ff055-e7a5-45ba-a735-6328311bfe6b)


