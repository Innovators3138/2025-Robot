package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PositionRobot extends Command {
    private static final LinearVelocity POSITION_VELOCITY = MetersPerSecond.of(0.25);
    private final RelativePosition m_direction;
    private final SwerveSubsystem m_swerveSubsystem;

    public PositionRobot(RelativePosition direction, SwerveSubsystem swerveSubsystem) {
        m_direction = direction;
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {
        var x = m_direction == RelativePosition.Forward
            ? POSITION_VELOCITY.in(MetersPerSecond)
            : m_direction == RelativePosition.Back
            ? -POSITION_VELOCITY.in(MetersPerSecond)
            : 0;
        
        var y = m_direction == RelativePosition.Right
            ? -POSITION_VELOCITY.in(MetersPerSecond)
            : m_direction == RelativePosition.Left
            ? POSITION_VELOCITY.in(MetersPerSecond)
            : 0;

        m_swerveSubsystem.drive(
            new Translation2d(x, y),
            0, false
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
