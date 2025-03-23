package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToLoader extends Command {
    // controls how far from the load the robot should be positioned
    private static final Transform2d ALIGNMENT_TRANSFORM = new Transform2d(
        Meters.of(0.5),
        Meters.of(0),
        Rotation2d.kZero
    );

    private final LoaderPosition m_loaderPosition;
    private final SwerveSubsystem m_swerveSubsystem;
    private Command m_driveToPoseCommand;

    public DriveToLoader(LoaderPosition loaderPosition, SwerveSubsystem swerveSubsystem) {
        m_loaderPosition = loaderPosition;
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        var target = m_loaderPosition == LoaderPosition.Left
            ? m_swerveSubsystem.getLeftLoaderPosition()
            : m_swerveSubsystem.getRightLoaderPosition();

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
}
