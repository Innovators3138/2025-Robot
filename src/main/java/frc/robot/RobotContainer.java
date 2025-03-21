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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.commands.ClimbComands.*;
import frc.robot.commands.*;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer
{
 
 
  GripperSubsystem m_GripperSubsystem = new GripperSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  private final CANdi armCANdi = new CANdi(34);
  private final CANdi elevatorCANdi = new CANdi(35);
  private final Arm m_arm = new Arm(armCANdi);
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
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
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(elevatorCANdi);
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem(elevatorCANdi);
  private static final String autoDefault = "Drive Forward";
  private static final String autoDoNothing = "Do Nothing";
  private static final String autoLeftSideL1 = "Left Side L1";
  private static final String autoCenterL1 = "Center L1";
  private static final String autoRightSideL1 = "Right Side L1";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  public RobotContainer()
  {
    
  
    
    
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
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    //driverXbox.x().whileTrue(drivebase.sysIdAngleMotorCommand());
    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper();
      driverXbox.rightBumper().onTrue(Commands.none());
    } else 
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      operatorXbox.x().onTrue(new SetToLevelOne(m_elevator, m_arm));
      operatorXbox.a().onTrue(new SetToLevelTwo(m_elevator, m_arm));
      operatorXbox.b().onTrue(new SetToLevelThree(m_elevator, m_arm));
      operatorXbox.y().onTrue(new SetToLevelFour(m_elevator, m_arm));

      //operatorXbox.rightStick().onTrue(new IntakeGamepiece(m_elevator, m_arm, m_GripperSubsystem).andThen(new WaitCommand(0.5)).andThen(new SetToLevelOne(m_elevator, m_arm)));
      operatorXbox.rightStick().onTrue(new IntakeGamepiece(m_elevator, m_arm, m_GripperSubsystem)); //run intake


      m_GripperSubsystem.setDefaultCommand(new RunGripper(m_GripperSubsystem, operatorXbox));
      //m_climbSubsystem.setDefaultCommand(new ClimbCommand(m_climbSubsystem, driverXbox, operatorXbox));

      //this uses trigger on driver controller to test the climb mechanism movements with touch sensitivity
      m_climbSubsystem.setDefaultCommand(new RunClimbTester(m_climbSubsystem, driverXbox));

      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().onTrue(new CloseIntake(m_intakeSubsystem).andThen(new DeactivateIntake(m_intakeSubsystem)));
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Left Side L1");
    //return drivebase.getAutonomousCommand("Center L1");
    //return drivebase.getAutonomousCommand("Right Side L1");
    //return drivebase.getAutonomousCommand("Do Nothing");
    
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
