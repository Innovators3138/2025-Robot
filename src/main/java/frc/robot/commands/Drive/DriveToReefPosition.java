package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class DriveToReefPosition extends Command {
    // controls how far from the reef the robot is positioned
    private static final Transform2d ALIGNMENT_TRANSFORM = new Transform2d(
        Meters.of(1),
        Meters.of(0),
        Rotation2d.k180deg
    );

    private final ReefPosition m_reefPosition;
    private final SwerveSubsystem m_swerveSubsystem;
    private Command m_driveToPoseCommand;

    public DriveToReefPosition(ReefPosition reefPosition, SwerveSubsystem swerveSubsystem) {
        m_reefPosition = reefPosition;
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        var target = getTarget();

        if (target.isEmpty()) {
            return;
        }

        var pose3d = m_swerveSubsystem.getTagPose(target.get().ID);
        if (pose3d.isEmpty()) {
            return;
        }
        
        var targetPose = pose3d.get().toPose2d()
            .transformBy(ALIGNMENT_TRANSFORM);

        m_driveToPoseCommand = m_swerveSubsystem.driveToPose(targetPose, VisionConstants.AUTO_DRIVE_VELOCITY, VisionConstants.AUTO_DRIVE_ACCELERATION);

        m_driveToPoseCommand.initialize();
    }

    @Override
    public void execute() {
        if (m_driveToPoseCommand == null) {
            return;
        }

        m_driveToPoseCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_driveToPoseCommand == null) {
            return;
        }

        m_driveToPoseCommand.end(interrupted);
        m_driveToPoseCommand = null;
    }

    @Override
    public boolean isFinished() {
        if (m_driveToPoseCommand == null) {
            return true;
        }

        return m_driveToPoseCommand.isFinished();
    }

    private Optional<AprilTag> getTarget() {
        if (m_reefPosition == ReefPosition.Nearest) {
            return m_swerveSubsystem.getNearestReefPosition();
        }

        var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        var tags = Vision.fieldLayout.getTags().stream();
        
        if (alliance == Alliance.Red) {
            switch (m_reefPosition) {
                case _12oClock:
                    return tags.filter(tag -> tag.ID == 10).findFirst();
                case _2oClock:
                    return tags.filter(tag -> tag.ID == 9).findFirst();
                case _4oClock:
                    return tags.filter(tag -> tag.ID == 8).findFirst();
                case _6oClock:
                    return tags.filter(tag -> tag.ID == 7).findFirst();
                case _8oClock:
                    return tags.filter(tag -> tag.ID == 6).findFirst();
                case _10oClock:
                    return tags.filter(tag -> tag.ID == 11).findFirst();
                default:
                    return Optional.empty();
            }
        } else {
            switch (m_reefPosition) {
                case _12oClock:
                    return tags.filter(tag -> tag.ID == 21).findFirst();
                case _2oClock:
                    return tags.filter(tag -> tag.ID == 22).findFirst();
                case _4oClock:
                    return tags.filter(tag -> tag.ID == 17).findFirst();
                case _6oClock:
                    return tags.filter(tag -> tag.ID == 18).findFirst();
                case _8oClock:
                    return tags.filter(tag -> tag.ID == 19).findFirst();
                case _10oClock:
                    return tags.filter(tag -> tag.ID == 20).findFirst();
                default:
                    return Optional.empty();
            }
        }
    }
}
