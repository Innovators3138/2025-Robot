package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Stop extends Command {
    private final SwerveSubsystem m_swerveSubsystem;

    public Stop(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {
        m_swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
