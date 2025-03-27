package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToTarget extends Command {
    private final TargetAlignment m_alignment;
    private final CommandXboxController m_controller;
    private final SwerveSubsystem m_swerveSubsystem;
    private final PIDController m_rotationController = new PIDController(0.3, 0, 0);
    private final PIDController m_strafeController = new PIDController(3, 0, 0);
    private final PIDController m_rangeController = new PIDController(3, 0, 0);
    private final double MAX_VELOCITY = MetersPerSecond.of(.5).magnitude();
    private final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(45);
    private int m_targetId = 0;
    private int m_count = 0;
    
    // note that this is the distance from the camera to the target, not the front bumper
    private final double IDEAL_RANGE = Units.inchesToMeters(24);

    private final DoublePublisher m_rotationOffsetPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/rotationOffset")
        .publish();

    private final DoublePublisher m_horizontalOffsetPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/horizontalOffset")
        .publish();

    private final DoublePublisher m_rangePublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/range")
        .publish();

    private final IntegerPublisher m_targetPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getIntegerTopic("vision/align/target")
        .publish();

    public AlignToTarget(TargetAlignment alignment, CommandXboxController controller, SwerveSubsystem swerveSubsystem) {
        m_alignment = alignment;
        m_controller = controller;
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_count = 0;

        // setup rotation controller
        m_rotationController.reset();
        m_rotationController.setSetpoint(0);
        m_rotationController.setTolerance(Units.degreesToRadians(10));

        // setup strafe controller
        m_strafeController.reset();

        m_strafeController.setSetpoint(
            m_alignment == TargetAlignment.Left
                ? Units.inchesToMeters(-6.5)
                : m_alignment == TargetAlignment.Right
                ? Units.inchesToMeters(6.5)
                : 0
        );

        m_strafeController.setTolerance(
            Units.inchesToMeters(0.5)
        );

        // setup range controller
        m_rangeController.reset();

        m_rangeController.setSetpoint(IDEAL_RANGE);

        m_rangeController.setTolerance(
            Units.inchesToMeters(1)
        );

        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.1);

        var target = m_swerveSubsystem.getBestReefTargetForAlignment();
        if (target.isPresent()) {
            m_targetId = target.get().fiducialId;
        }
    }

    @Override
    public void execute() {
        m_count++;
        
        var target = m_targetId == 0
            ? m_swerveSubsystem.getBestReefTargetForAlignment()
            : m_swerveSubsystem.getTrackedTarget(m_targetId);

        if (target.isEmpty()) {
            m_swerveSubsystem.drive(Translation2d.kZero, 0, false);
            return;
        }

        if (m_targetId == 0) {
            m_targetId = target.get().fiducialId;
        }

        var transform = target.get().getBestCameraToTarget();
        var currentRange = transform.getTranslation().getX();
        var currentOffset = transform.getTranslation().getY();
        var rotationOffset = transform.getRotation()
            .toRotation2d()
            .rotateBy(Rotation2d.k180deg)
            .getDegrees();

        m_targetPublisher.set(target.get().fiducialId);
        m_rotationOffsetPublisher.set(rotationOffset);
        m_horizontalOffsetPublisher.set(currentOffset);
        m_rangePublisher.set(currentRange);

        var strafeVelocity = MathUtil.clamp(
            m_strafeController.calculate(currentOffset),
            -MAX_VELOCITY,
            MAX_VELOCITY);

        var rangeVelocity = MathUtil.clamp(
            m_rangeController.calculate(currentRange),
            -MAX_VELOCITY,
            MAX_VELOCITY);

        var rotationVelocity = MathUtil.clamp(
            m_rotationController.calculate(rotationOffset),
            -MAX_ANGULAR_VELOCITY,
            MAX_ANGULAR_VELOCITY);

        m_swerveSubsystem.drive(
            new Translation2d(-rangeVelocity, -strafeVelocity),
            -rotationVelocity,
            false
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        m_swerveSubsystem.drive(
            new Translation2d(0, 0),
            0, false
        );

        m_targetId = 0;
    }

    @Override
    public boolean isFinished() {
        if (m_count > 150) {
            return true;
        }
        
        return m_strafeController.atSetpoint() &&
            m_rotationController.atSetpoint() &&
            m_rangeController.atSetpoint();
    }
}
