// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;
import static edu.wpi.first.units.Units.*;

import java.security.PublicKey;

import edu.wpi.first.units.measure.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum Level {
    STOWED,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final double NOMINAL_VOLTAGE = 12.0;
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final String NEO_CURRENT_LIMIT = null;
public static final int LOWER_INTAKE_BAR_MOTOR_ID = 0;
public static final int UPPER_INTAKE_BAR_MOTOR_ID = 1;
public static final boolean LOWER_INTAKE_BAR_INVERT = true;
public static final boolean UPPER_INTAKE_BAR_INVERT = false;
public static final double UPPER_INTAKE_BAR_SPEED = 0.5;
public static final double LOWER_INTAKE_BAR_SPEED = 0.5;

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class ArmConstants
  {
    public static final Angle ARM_INTAKE_ANGLES[] = {Rotations.of(0.040), Rotations.of(0.122)};
    public static final Angle ARM_L1_ANGLES[] = {Rotations.of(-0.056), Rotations.of(-0.005)};
    public static final Angle ARM_L2_ANGLES[] = {Rotations.of(-0.060), Rotations.of(0.204)};
    public static final Angle ARM_L3_ANGLES[] = {Rotations.of(-0.445), Rotations.of(0.133)};
    public static final Angle ARM_L4_ANGLES[] = {Rotations.of(-0.445), Rotations.of(0.133)};
    public static final Angle ARM_STOWED_ANGLES[] = {Rotations.of(0.03), Rotations.of(0.112)};

    public static final int SHOULDER_ENCODER_SIGNAL = 1;
    public static final int WRIST_ENCODER_SIGNAL = 2;

    public static final Angle SHOULDER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = Rotations.of(0.3);
    public static final Angle WRIST_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = Rotations.of(0.5);

    public static final Angle SHOULDER_ABSOLUTE_SENSOR_OFFSET = Rotations.of(0.552);
    public static final Angle WRIST_ABSOLUTE_SENSOR_OFFSET = Rotations.of(0.012);

    public static final boolean SHOULDER_ENCODER_IS_INVERTED = true;
    public static final boolean WRIST_ENCODER_IS_INVERTED = false;

    public static final AngularVelocity SHOULDER_IDLE_SPEED_REQUIRED = RotationsPerSecond.of(0.02);
    public static final AngularVelocity WRIST_IDLE_SPEED_REQUIRED = RotationsPerSecond.of(0.02);

    public static final double WRIST_MAX_ROTATION = 0.650;

    public static final double SHOULDER_P = 12;//20.0;
    public static final double SHOULDER_I = 5.0;
    public static final double SHOULDER_D = 0;
    public static final double SHOULDER_FF = 0;
    public static final double SHOULDER_IZ = 0.15;

    public static final double SHOULDER_KG = 1.07;
    public static final double SHOULDER_KV = 1.69;
    public static final double SHOULDER_KA = 0.06;

    public static final double WRIST_KG = 0.05;
    public static final double WRIST_KV = 0.98;
    public static final double WRIST_KA = 0.00;

    

    public static final double WRIST_P = 8;//26.0;
    public static final double WRIST_I = 1.0;
    public static final double WRIST_D = 0.5;
    public static final double WRIST_FF = 0;
    public static final double WRIST_IZ = 10.0;

    public static final double SHOULDER_CONVERSION_FACTOR = 40.0 / 80.0;

    public static final int SHOULDER_MOTOR_CURRENT_LIMIT = 40;
    public static final int WRIST_MOTOR_CURRENT_LIMIT = 20;
    public static final double SHOULDER_MOTOR_RAMP_RATE = 0.25;
    public static final double WRIST_MOTOR_RAMP_RATE = 0.25;
    public static final boolean SHOULDER_MOTOR_IS_INVERTED = false;
    public static final boolean WRIST_MOTOR_IS_INVERTED =  false;

    public static final Angle SHOULDER_SAFE_STOWED_RANGE[] = {Degrees.of(0), Degrees.of(1)};

  }

  public static final class ClimberConstants {
    public static final Angle CLIMB_ANGLE = Rotations.of(0.00);
    public static final Angle STOW_ANGLE = Rotations.of(0.25);

    public static final boolean MOTOR_IS_INVERTED = false;
    public static final boolean ENCODER_IS_INVERTED = false;
    public static final double ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 0.3;
    public static final double ABSOLUTE_SENSOR_OFFSET = 0.225;
  }

  public static final class ElevatorConstants {
    public static final Distance MINIMUM_SAFE_HEIGHT = Inches.of(0.5);
    public static final Distance MINIMUM_SHOULDER_MOVE_SAFE_HEIGHT = Inches.of(10.0); 
    public static final Distance MAXIMUM_SAFE_HEIGHT = Inches.of(27.0);

    public static final Distance ELEVATOR_INTAKE_HEIGHT = Meters.of(0);
    public static final Distance ELEVATOR_L1_HEIGHT = Meters.of(0.0);
    public static final Distance ELEVATOR_L2_HEIGHT = Meters.of(0.459);
    public static final Distance ELEVATOR_L3_HEIGHT = Meters.of(0.0);
    public static final Distance ELEVATOR_L4_HEIGHT = Meters.of(0.632);
    public static final Distance ELEVATOR_STOWED_HEIGHT = Meters.of(0.0);

  public static final Distance ELEVATOR_INITIAL_HEIGHT = Meters.of(0.0);
  public static final Distance ELEVATOR_MAX_HEIGHT = Meters.of(0.666);
  public static final boolean ELEVATOR_MOTOR_IS_INVERTED = true;
  public static final boolean ELEVATOR_ENCODER_IS_INVERTED = true;
  public static final double ELEVATOR_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 0.95;
  public static final double ELEVATOR_ABSOLUTE_SENSOR_OFFSET = 0.225;
  public static final double ELEVATOR_P = 12;
  public static final double ELEVATOR_I = 2.0;
  public static final double ELEVATOR_D = 0.1;
  public static final double ELEVATOR_FF = 0;
  public static final double ELEVATOR_IZ = 0.1;

  public static final double ELEVATOR_KG = 0.48;
  public static final double ELEVATOR_KV = 2.66;
  public static final double ELEVATOR_KA = 0.05;

  public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 40;
  public static final double ELEVATOR_MOTOR_RAMP_RATE = 0.25;
  public static final LinearVelocity ELEVATOR_MAX_VELOCITY = MetersPerSecond.of(1.0);

  public static final Distance ELEVATOR_DRUM_DIAMETER = Inches.of(1.432);
  public static final Distance ELEVATOR_CONVERSION_FACTOR = ELEVATOR_DRUM_DIAMETER.times(Math.PI * 64.0 / 24.0 * 64.0 / 24.0); // Distance per Magnet Rotation
  public static final Distance ELEVATOR_THRESHOLD = Inches.of(0.125);

  
  
  }

  public static final class GripperConstants {
    public static final double LIGHT_SENSOR_THRESHOLD = 0.55; // 0.55 volts
    public static final double GRIPPER_VOLTAGE_COEFFICIENT = 0.5484375;
    public static final double GRIPPER_INTAKE_SPEED = 0.5;
    public static final double TRIGGER_DEADZONE = 0.05;
  }
}
