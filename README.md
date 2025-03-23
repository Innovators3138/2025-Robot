# 2025-Robot

### CAN ID Configuration ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| roborio | core | 0-9 | master (no ID) |
| pdh | core | 0-9 | 0
| front_left_drive | SparkMax | 10-29 | 10 |
| front_left_steer | SparkMax | 10-29 | 11 |
| front_right_drive | SparkMax | 10-29 | 12 |
| front_right_steer | SparkMax | 10-29 | 13 |
| back_left_drive | SparkMax | 10-29 | 14 |
| back_left_steer | SparkMax | 10-29 | 15 |
| back_right_drive | SparkMax | 10-29 | 16 |
| back_right_steer | SparkMax | 10-29 | 17 |
| shoulder_motor | SparkFlex | 10-29 | 18 | 
| wrist_motor | SparkMax | 10-29 | 19 |
| left_intake_motor | SparkMax | 10-29 | 20 | 
| right_intake_motor | SparkMax | 10-29 | 21 |
| elevator_motor | SparkFlex | 10-29 | 22 |
| front_left_encoder | CANCoder | 30-39 | 30 |
| front_right_encoder | CANCoder | 30-39 | 31 |
| back_left_encoder | CANCoder | 30-39 | 32 |
| back_right_encoder | CANCoder | 30-39 | 33 |
| arm_candi | CANdi | 30-39 | 34 |


### Analog Input ID Configuration ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| high_pressure_transducer | AnalogInput | 0-3 | 0 |
| working_pressure_transducer | AnalogInput | 0-3 | 1 |


### Digital Input ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| elevator_encoder | DutyCycleEncoder | 0-3 | 0 |

### Solenoids ###
| Device | Class | Range | ID |
|:-|:-:|:-:|:-:|
| left_intake_solenoid | DoubleSolenoid | 0-15 | 0, 1 |
| right_intake_solenoid | DoubleSolenoid | 0-15 | 2, 3 |
| left_intake_solenoid | DoubleSolenoid | 0-15 | 0, 1 |

### Drive Controls ###
- **Left Stick** - field oriented robot movement
- **Right Stick** - auto drive to a specific reef position
  - Use path planner to driver the robot to a specific reef position, and rotate the bot to
  face the april tag at that position.
  - From the driver's perspective, imagine the reef is a clock.  Push the stick up (12 o'clock), and the robot
  will autonomously drive to the reef position nearest to the center of the field.  Push the stick down (6 o'clock)
  and the robot will autonomously drive to the reef position closest to our alliances side of the field.
- **Left Trigger/Right trigger** - robot oriented rotation
  - left trigger rotates the robot counterclockwise, right trigger rotates the robot clockwise
- **D-Pad** - fine grained robot oriented movement
  - Up on the d-pad moves the robot forward, down backward, etc.
- **A button** - auto drive to nearest reef position
  - Use the path planner to drive the robot to the nearest reef position, and rotate the bot to
  face the april tag at that position.
- **B button** - stop the robot
  - Stops the robot, interrupting any running commands
- **X button** - align robot to left scoring position
- **Y button** - align robot to right scoring position
- **Left Bumper** - auto drive to left loading station
  - Auto generate a path to take the robot to the left loading station (drivers left hand side), rotated so that the bot is ready to be loaded
  - "alliance aware", meaning left and right depends on which side of the field you are one.
- **Right Bumper** - auto drive to right loading station
  - Auto generate a path to take the robot to the right loading station (drivers right hand side), rotated so that the bot is ready to be loaded
  - "alliance aware", meaning left and right depends on which side of the field you are one.
