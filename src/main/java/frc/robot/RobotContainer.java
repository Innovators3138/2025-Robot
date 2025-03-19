// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANdi;
import com.fasterxml.jackson.databind.introspect.WithMember;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.commands.ClimbComands.ClimbCommand;
import frc.robot.commands.ClimbComands.StopClimbing;
import frc.robot.commands.Drive.DriveToLoader;
import frc.robot.commands.Drive.DriveToReefPosition;
import frc.robot.commands.Drive.LoaderPosition;
import frc.robot.commands.Drive.PositionRobot;
import frc.robot.commands.Drive.RelativePosition;
import frc.robot.commands.Drive.AlignToTarget;
import frc.robot.commands.Drive.RotateRobot;
import frc.robot.commands.Drive.RotationDirection;
import frc.robot.commands.Drive.ReefPosition;
import frc.robot.commands.Drive.RotateRobot;
import frc.robot.commands.Drive.Stop;
import frc.robot.commands.Drive.TargetAlignment;
import frc.robot.commands.IntakeGamepiece;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.MoveShoulder;
import frc.robot.commands.RunGripper;
import frc.robot.commands.MoveShoulderAndWrist;
import frc.robot.commands.SetToLevelOne;
import frc.robot.commands.SetToLevelTwo;
import frc.robot.commands.SetToLevelThree;
import frc.robot.commands.SetToLevelFour;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import java.io.File;
import swervelib.SwerveInputStream;

import frc.robot.subsystems.climb.ClimbSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

  GripperSubsystem m_GripperSubsystem = new GripperSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  private final CANdi armCANdi = new CANdi(34);
  private final CANdi elevatorCANdi = new CANdi(35);
  private final Arm m_arm = new Arm(armCANdi);
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .scaleRotation(0.001)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(elevatorCANdi);
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private static final String autoDefault = "Drive Forward";
  private static final String autoDoNothing = "Do Nothing";
  private static final String autoLeftSideL1 = "Left Side L1";
  private static final String autoCenterL1 = "Center L1";
  private static final String autoRightSideL1 = "Right Side L1";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    m_chooser.setDefaultOption("Drive Forward", autoDefault);
    m_chooser.addOption("Do Nothing", autoDoNothing);
    m_chooser.addOption("Left Side L1", autoLeftSideL1);
    m_chooser.addOption("Center L1", autoCenterL1);
    m_chooser.addOption("Right Side L1", autoRightSideL1);

    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    setupRightStickAuton();
    
    driverXbox.a().onTrue(
      new SequentialCommandGroup(
        new DriveToReefPosition(ReefPosition.Nearest, drivebase),
        new AlignToTarget(TargetAlignment.Center, driverXbox, drivebase)
      )
    );

    driverXbox.b().onTrue(new Stop(drivebase));

    driverXbox.x()
      .onTrue(new AlignToTarget(TargetAlignment.Left, driverXbox, drivebase));

    driverXbox.y()
      .onTrue(new AlignToTarget(TargetAlignment.Right, driverXbox, drivebase));

    driverXbox.rightBumper().onTrue(new DriveToLoader(LoaderPosition.Right, drivebase));
    driverXbox.leftBumper().onTrue(new DriveToLoader(LoaderPosition.Left, drivebase));

    driverXbox.povLeft().whileTrue(new PositionRobot(RelativePosition.Left, drivebase));
    driverXbox.povRight().whileTrue(new PositionRobot(RelativePosition.Right, drivebase));
    driverXbox.povUp().whileTrue(new PositionRobot(RelativePosition.Forward, drivebase));
    driverXbox.povDown().whileTrue(new PositionRobot(RelativePosition.Back, drivebase));

    driverXbox.rightTrigger()
      .whileTrue(new RotateRobot(RotationDirection.Clockwise, drivebase));

    driverXbox.leftTrigger()
      .whileTrue(new RotateRobot(RotationDirection.CounterClockwise, drivebase));
    
    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    if (DriverStation.isTest()) {
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper();
      driverXbox.rightBumper().onTrue(Commands.none());
    } else 
    {
      //drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      //operatorXbox.y().onTrue(new MoveShoulder(arm, Rotations.of(-0.171)).repeatedly().andThen(new MoveWrist(arm, Rotations.of(0.112)).repeatedly()));
      
      /*operatorXbox.b().onTrue(new MoveShoulder(arm, ArmConstants.ARM_INTAKE_ANGLES[0]).repeatedly());
      operatorXbox.a().onTrue(new MoveShoulder(arm, ArmConstants.ARM_L1_ANGLES[0]).repeatedly());
      operatorXbox.x().onTrue(new MoveShoulder(arm, ArmConstants.ARM_L2_ANGLES[0]).repeatedly());
      operatorXbox.y().onTrue(new MoveShoulder(arm, ArmConstants.ARM_L3_ANGLES[0]).repeatedly());*/
      //operatorXbox.b().onTrue(new MoveShoulderAndWrist(m_arm, ArmConstants.ARM_INTAKE_ANGLES[0], ArmConstants.ARM_INTAKE_ANGLES[1]).repeatedly());
      //operatorXbox.a().onTrue(new MoveShoulderAndWrist(m_arm, ArmConstants.ARM_L1_ANGLES[0], ArmConstants.ARM_L1_ANGLES[1]).repeatedly());
      //operatorXbox.x().onTrue(new MoveShoulderAndWrist(m_arm, ArmConstants.ARM_L2_ANGLES[0], ArmConstants.ARM_L2_ANGLES[1]).repeatedly());
      //operatorXbox.y().onTrue(new MoveShoulderAndWrist(m_arm, ArmConstants.ARM_L3_ANGLES[0], ArmConstants.ARM_L3_ANGLES[1]).repeatedly());

      operatorXbox.x().onTrue(new SetToLevelOne(m_elevator, m_arm).repeatedly());
      operatorXbox.a().onTrue(new SetToLevelTwo(m_elevator, m_arm).repeatedly());
      operatorXbox.b().onTrue(new SetToLevelThree(m_elevator, m_arm).repeatedly());
      operatorXbox.y().onTrue(new SetToLevelFour(m_elevator, m_arm).repeatedly());

      operatorXbox.rightStick().onTrue(new IntakeGamepiece(m_elevator, m_arm, m_GripperSubsystem).repeatedly());
      
      //operatorXbox.x().onTrue(new MoveWrist(arm, Rotations.of(0.112)).repeatedly());
      //operatorXbox.b().onTrue(new MoveWrist(arm, Rotations.of(0.002)).repeatedly());
      //operatorXbox.y().onTrue(new RunGripper(m_GripperSubsystem, 0.3 ).repeatedly());
      m_GripperSubsystem.setDefaultCommand(new RunGripper(m_GripperSubsystem, operatorXbox));
      //operatorXbox.leftBumper().onTrue(new RunGripper(m_GripperSubsystem, 0.1).repeatedly());
      //operatorXbox.rightBumper().onTrue(new RunGripper(m_GripperSubsystem, 1).repeatedly());
      //operatorXbox.leftBumper().onFalse(new RunGripper(m_GripperSubsystem, 0.0).repeatedly());
      //operatorXbox.rightBumper().onFalse(new RunGripper(m_GripperSubsystem, 0.0).repeatedly());
      //operatorXbox.a().onTrue(new SetToLevelOne(m_elevator, arm));
      //operatorXbox.x().onTrue(new SetToLevelTwo(m_elevator, arm));
      //operatorXbox.b().onTrue(new SetToLevelThree(m_elevator, arm));
      //operatorXbox.y().onTrue(new SetToLevelFour(m_elevator, arm));

      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().onTrue(new CloseIntake(m_intakeSubsystem).andThen(new
      // DeactivateIntake(m_intakeSubsystem)));
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new DriveToReefPosition(ReefPosition._2oClock, drivebase),
      new AlignToTarget(TargetAlignment.Right, driverXbox, drivebase),
      new WaitCommand(Seconds.of(3)),
      new DriveToLoader(LoaderPosition.Right, drivebase),
      new WaitCommand(Seconds.of(1)),
      new DriveToReefPosition(ReefPosition._2oClock, drivebase),
      new AlignToTarget(TargetAlignment.Left, driverXbox, drivebase),
      new WaitCommand(Seconds.of(3)),
      new DriveToLoader(LoaderPosition.Right, drivebase),
      new WaitCommand(Seconds.of(1)),
      new DriveToReefPosition(ReefPosition._4oClock, drivebase),
      new AlignToTarget(TargetAlignment.Right, driverXbox, drivebase),
      new WaitCommand(Seconds.of(3)),
      new DriveToLoader(LoaderPosition.Right, drivebase)

    );
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("Left Side L");
    // return drivebase.getAutonomousCommand("Center L1");
    // return drivebase.getAutonomousCommand("Right Side L1");
    // return drivebase.getAutonomousCommand("Do Nothing");

  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  private void setupRightStickAuton() {
    // 12 o'clock (up)
    driverXbox.axisLessThan(Axis.kRightY.value, -0.95)
      .onTrue(
        new SequentialCommandGroup(
          new DriveToReefPosition(ReefPosition._12oClock, drivebase),
          new AlignToTarget(TargetAlignment.Center, driverXbox, drivebase)
        )
      );

    // 2 o'clock (right + up)
    driverXbox
      .axisGreaterThan(Axis.kRightX.value, 0.5)
      .and(driverXbox.axisLessThan(Axis.kRightY.value, -0.5))
      .onTrue(
        new SequentialCommandGroup(
          new DriveToReefPosition(ReefPosition._2oClock, drivebase),
          new AlignToTarget(TargetAlignment.Center, driverXbox, drivebase)
        )
      );

    // 4 o'clock (right + down)
    driverXbox
      .axisGreaterThan(Axis.kRightX.value, 0.5)
      .and(driverXbox.axisGreaterThan(Axis.kRightY.value, 0.5))
      .onTrue(
        new SequentialCommandGroup(
          new DriveToReefPosition(ReefPosition._4oClock, drivebase),
          new AlignToTarget(TargetAlignment.Center, driverXbox, drivebase)
        )
      );

    // 6 o'clock (down)
    driverXbox
      .axisGreaterThan(Axis.kRightY.value, 0.95)
      .onTrue(
        new SequentialCommandGroup(
          new DriveToReefPosition(ReefPosition._6oClock, drivebase),
          new AlignToTarget(TargetAlignment.Center, driverXbox, drivebase)
        )
      );
      
    // 8 o'clock (left + down)
    driverXbox
      .axisLessThan(Axis.kRightX.value, -0.5)
      .and(driverXbox.axisGreaterThan(Axis.kRightY.value, 0.5))
      .onTrue(
        new SequentialCommandGroup(
          new DriveToReefPosition(ReefPosition._8oClock, drivebase),
          new AlignToTarget(TargetAlignment.Center, driverXbox, drivebase)
        )
      );

    // 10 o'clock (left + up)
    driverXbox
      .axisLessThan(Axis.kRightX.value, -0.5)
      .and(driverXbox.axisLessThan(Axis.kRightY.value, -0.5))
      .onTrue(
        new SequentialCommandGroup(
          new DriveToReefPosition(ReefPosition._10oClock, drivebase),
          new AlignToTarget(TargetAlignment.Center, driverXbox, drivebase)
        )
      );
  }
}
