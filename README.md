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
