package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class BumpReef extends Command {
    private final double CAMERA_TO_BUMPER = Units.inchesToMeters(9);
    private final LinearVelocity BUMP_VELOCITY = MetersPerSecond.of(0.25);
    private final SwerveSubsystem m_swerveSubsystem;
    private Pose2d m_bumpPose;

    public BumpReef(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        var target = m_swerveSubsystem.getBestReefTargetForAlignment();
        if (target.isEmpty()) {
            return;
        }

        var targetTransform = target.get().getBestCameraToTarget();
        var robotPose = m_swerveSubsystem.getPose();
        var driveDistance = targetTransform.getX() - CAMERA_TO_BUMPER;

        m_bumpPose = robotPose
            .transformBy(
                new Transform2d(
                    new Translation2d(
                        driveDistance,
                        0
                    ),
                    Rotation2d.kZero
                )
            );
    }

    @Override
    public void execute() {
        if (m_bumpPose == null) {
            return;
        }

        m_swerveSubsystem.drive(
            new Translation2d(BUMP_VELOCITY.in(MetersPerSecond), 0),
            0, false
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        if (m_bumpPose == null) {
            return;
        }

        m_bumpPose = null;
    }

    @Override
    public boolean isFinished() {
        if (m_bumpPose == null) {
            return true;
        }

        var distance = m_bumpPose.getTranslation().getDistance(m_swerveSubsystem.getPose().getTranslation());
        return Math.abs(distance) < 0.01;
    }
}
