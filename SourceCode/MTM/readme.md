# General Functionality MTM
For both MTMs of the training console, the program is required to read the data from a total of six absolute magnetic encoders, two IMUs and two Hall effect sensors. Whereas, for the received data, a distinction must be made between the movement data of the left and right arm. This results in a total of eight sensor values received per MTM which must be communicated to the computer I/O where they are processed into appropriate joint values for the PSM. 
 
 
## MT_MTM.ino
The main control file for the MTM system.
- Integrating all subsystems and coordinating their operations.
- Handling communication with the central controller (Arduino Mega).
- Managing the overall workflow of the MTM system.

### spikeDetection.ino
This file handles the detection of spikes in the movement data, which might indicate rapid or sudden movements. Key functionalities include:
- Variables to store previous and current IMU (Inertial Measurement Unit) readings.
- Spike detection logic to identify abrupt changes in movement.
- Implementation of counters and difference calculations to process the movement data.

### Initial_preVal_def.ino
This file initializes various variables and sets up default values for the system. Key functionalities include:
- Defining initial positions and orientations for the robot.
- Setting up initial encoder values and IMU addresses.
- Initializing kinematic variables for both MTM and PSM.

### KinematicCalc_L.ino and KinematicCalc_R.ino
These files perform kinematic calculations for the left and right sides of the robot, respectively. Key functionalities include:
- Calculating joint angles and positions based on encoder and IMU data.
- Updating the kinematic chain for the MTM to ensure accurate movement detection.

### LPFilter.ino and LPFilter_Encoder.ino
These files implement low-pass filters to smooth the data received from the sensors. Key functionalities include:
- Filtering IMU data to remove noise and ensure smooth movement detection.
- Filtering encoder data to ensure accurate position readings.

### readEncoder.ino
This file contains the code to read data from the encoders. Key functionalities include:
- Reading the position data from the encoders connected to the robot joints.
- Processing the encoder signals to extract meaningful position information.

### OverFlowDetection.ino
This file handles the detection of overflow conditions in the sensor data. Key functionalities include:
- Monitoring sensor values to detect overflows.
- Implementing corrective actions to handle overflow conditions and ensure continuous operation.

### readIMU.ino
This file contains the code to read data from the IMUs. Key functionalities include:
- Reading quaternion data from the IMUs to determine the orientation of the robot.
- Handling FIFO (First In, First Out) buffers to manage IMU data efficiently.

### setupIMU.ino
This file sets up the IMUs (Inertial Measurement Units) for use in the MTM system. Key functionalities include:
- Initializing IMU sensors.
- Configuring IMU settings and calibration parameters.
- Ensuring the IMUs are ready for accurate orientation tracking.

### parseData.ino
This file contains the logic to parse incoming data strings. Key functionalities include:
- Tokenizing received data strings.
- Converting tokenized strings into numerical values.
- Storing parsed data into appropriate variables for further processing.

### recvWithStartEndMarkers.ino
This file implements the functionality to receive data with defined start and end markers. Key functionalities include:
- Detecting start and end markers in incoming serial data.
- Extracting and storing data between the markers.
- Setting flags to indicate new data availability for processing.
 
 
 

