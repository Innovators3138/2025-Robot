package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateRobot extends Command {
    private static final AngularVelocity ANGULAR_VELOCITY = RadiansPerSecond.of(0.5);
    private final RotationDirection m_direction;
    private final SwerveSubsystem m_swerveSubsystem;

    public RotateRobot(RotationDirection direction, SwerveSubsystem swerveSubsystem) {
        m_direction = direction;
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {
        var rotation = m_direction == RotationDirection.Clockwise
            ? -ANGULAR_VELOCITY.in(RadiansPerSecond)
            : ANGULAR_VELOCITY.in(RadiansPerSecond);

        m_swerveSubsystem.drive(Translation2d.kZero, rotation, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
