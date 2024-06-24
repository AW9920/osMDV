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

