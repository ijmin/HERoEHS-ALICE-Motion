[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE
/dev/ttyUSB0 | 3000000  | r_knee_pitch 


[ device info ]
## TYPE   | PORT NAME    | ID   | MODEL         | PROTOCOL | DEV NAME           | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 |  1   | MX-106-1-1    | 2.0      | l_shoulder_pitch  | present_position
dynamixel | /dev/ttyUSB0 |  2   | MX-106-1-2    | 2.0      | r_shoulder_pitch  | present_position

#dynamixel | /dev/ttyUSB0 |  3   | MX-106-1-3     | 2.0      | l_shoulder_roll   | present_position
#dynamixel | /dev/ttyUSB0 |  4   | MX-106-1-4     | 2.0      | r_shoulder_roll   | present_position
#dynamixel | /dev/ttyUSB0 |  5   | MX-64-1-5      | 2.0      | l_elbow_pitch     | present_position
#dynamixel | /dev/ttyUSB0 |  6   | MX-64-1-6      | 2.0      | r_elbow_pitch     | present_position

dynamixel | /dev/ttyUSB0 |  7   | MX-64-1-7      | 2.0      | head_pitch         | present_position
dynamixel | /dev/ttyUSB0 |  8   | MX-64-1-8      | 2.0      | head_yaw           | present_position

#dynamixel | /dev/ttyUSB0 |  9   | MX-106-1-9     | 2.0      | waist_yaw          | present_position
#dynamixel | /dev/ttyUSB0 | 10   | MX-106-1-10    | 2.0      | waist_pitch        | present_position

dynamixel | /dev/ttyUSB0 | 11   | H54P-100-S500-R-1-11   | 2.0      | l_hip_pitch        | present_position
dynamixel | /dev/ttyUSB0 | 12   | H54P-100-S500-R-1-12   | 2.0      | r_hip_pitch        | present_position
dynamixel | /dev/ttyUSB0 | 13   | H54P-100-S500-R-1-13   | 2.0      | l_hip_roll         | present_position
dynamixel | /dev/ttyUSB0 | 14   | H54P-100-S500-R-1-14   | 2.0      | r_hip_roll         | present_position
dynamixel | /dev/ttyUSB0 | 15   | XH540-V270-1-15        | 2.0      | l_hip_yaw          | present_position
dynamixel | /dev/ttyUSB0 | 16   | XH540-V270-1-16        | 2.0      | r_hip_yaw          | present_position

dynamixel | /dev/ttyUSB0 | 17   | H54P-100-S500-R-1-17   | 2.0      | l_knee_pitch       | present_position
dynamixel | /dev/ttyUSB0 | 18   | H54P-100-S500-R-1-18   | 2.0      | r_knee_pitch       | present_position
dynamixel | /dev/ttyUSB0 | 19   | H54P-100-S500-R-1-19   | 2.0      | l_ankle_pitch      | present_position
dynamixel | /dev/ttyUSB0 | 20   | H54P-100-S500-R-1-20   | 2.0      | r_ankle_pitch      | present_position
dynamixel | /dev/ttyUSB0 | 21   | H54P-100-S500-R-1-21   | 2.0      | l_ankle_roll       | present_position
dynamixel | /dev/ttyUSB0 | 22   | H54P-100-S500-R-1-22   | 2.0      | r_ankle_roll       | present_position
