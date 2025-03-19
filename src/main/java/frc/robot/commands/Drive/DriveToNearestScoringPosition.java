package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToNearestScoringPosition extends Command {
    // controls how far from the reef the robot is positioned
    private static final Transform2d ALIGNMENT_TRANSFORM = new Transform2d(
        Meters.of(1),
        Meters.of(0),
        Rotation2d.k180deg
    );

    private final SwerveSubsystem m_swerveSubsystem;
    private AprilTag m_target;
    private Command m_driveToPoseCommand;

    public DriveToNearestScoringPosition(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_target = m_swerveSubsystem.getNearestScoringPosition();

        if (m_target == null) {
            return;
        }

        var pose3d = m_swerveSubsystem.getTagPose(m_target);
        if (pose3d.isEmpty()) {
            return;
        }
        
        var targetPose = pose3d.get().toPose2d()
            .transformBy(ALIGNMENT_TRANSFORM);

        m_driveToPoseCommand = m_swerveSubsystem.driveToPose(targetPose);

        m_driveToPoseCommand.initialize();
    }

    @Override
    public void execute() {
        m_driveToPoseCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveToPoseCommand.end(interrupted);

        m_target = null;
        m_driveToPoseCommand = null;
    }

    @Override
    public boolean isFinished() {
        if (m_driveToPoseCommand == null) {
            return true;
        }

        return m_driveToPoseCommand.isFinished();
    }
}
