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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final double NOMINAL_VOLTAGE = 12.0;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

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
    public static final double ARM_S1_ANGLES[] = {30.0, 30.0};
    public static final double ARM_S2_ANGLES[] = {30.0, 30.0};
    public static final double ARM_S3_ANGLES[] = {30.0, 30.0};
    public static final double ARM_S4_ANGLES[] = {30.0, 30.0};
    public static final double ARM_S5_ANGLES[] = {-150.0, 0.0};

    public static final int SHOULDER_ENCODER_SIGNAL = 1;
    public static final int WRIST_ENCODER_SIGNAL = 2;

    public static final Angle SHOULDER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = Degrees.of(0.0);
    public static final Angle WRIST_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = Degrees.of(0.0);

    public static final Angle SHOULDER_ABSOLUTE_SENSOR_OFFSET = Degrees.of(0.0);
    public static final Angle WRIST_ABSOLUTE_SENSOR_OFFSET = Degrees.of(0.0);

    public static final boolean SHOULDER_ENCODER_IS_INVERTED = false;
    public static final boolean WRIST_ENCODER_IS_INVERTED = false;

    public static final double SHOULDER_P = 1.0;
    public static final double SHOULDER_I = 0;
    public static final double SHOULDER_D = 0;
    public static final double SHOULDER_FF = 0;
    public static final double SHOULDER_IZ = 10.0;

    public static final double SHOULDER_KG = 1.07;
    public static final double SHOULDER_KV = 1.69;
    public static final double SHOULDER_KA = 0.06;

    public static final double WRIST_KG = 0.05;
    public static final double WRIST_KV = 0.98;
    public static final double WRIST_KA = 0.00;

    

    public static final double WRIST_P = 1.0;
    public static final double WRIST_I = 0;
    public static final double WRIST_D = 0;
    public static final double WRIST_FF = 0;
    public static final double WRIST_IZ = 10.0;

    public static final double CONVERSION_FACTOR = 1024.0 / 360.0;
    public static final double SHOOTER_COLLISION_ANGLE = 100;

    public static final int SHOULDER_MOTOR_CURRENT_LIMIT = 40;
    public static final int WRIST_MOTOR_CURRENT_LIMIT = 20;
    public static final double SHOULDER_MOTOR_RAMP_RATE = 0.25;
    public static final double WRIST_MOTOR_RAMP_RATE = 0.25;
    public static final boolean SHOULDER_MOTOR_IS_INVERTED = false;
    public static final boolean WRIST_MOTOR_IS_INVERTED =  false;

    public static final double SHOULDER_MAX_VELOCITY_DEG_PER_SEC = 100.0;
  }
}
