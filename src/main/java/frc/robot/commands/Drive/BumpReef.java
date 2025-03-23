package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class BumpReef extends Command {
    private final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(10);
    private final LinearVelocity BUMP_VELOCITY = MetersPerSecond.of(0.25);
    private final SwerveSubsystem m_swerveSubsystem;
    private final PIDController m_headingController = new PIDController(0.001, 0, 0);
    private int m_count;
    private double m_originalHeading;

    public BumpReef(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_count = 0;
        m_originalHeading = m_swerveSubsystem.getRawHeading().getDegrees();
        m_headingController.reset();
    }

    @Override
    public void execute() {
        m_count++;
        
        var currentHeading = m_swerveSubsystem.getRawHeading().getDegrees();
        var headingDrift = m_originalHeading - currentHeading;
        if (headingDrift < -180) {
            currentHeading = currentHeading - 360;
            headingDrift = m_originalHeading - currentHeading;
        }

        if (headingDrift > 180) {
            currentHeading = currentHeading + 360;
            headingDrift = m_originalHeading - currentHeading;
        }

        var angularVelocity = MathUtil.clamp(
            m_headingController.calculate(headingDrift),
            -MAX_ANGULAR_VELOCITY.in(DegreesPerSecond),
            MAX_ANGULAR_VELOCITY.in(DegreesPerSecond));

        m_swerveSubsystem.drive(
            new Translation2d(
                BUMP_VELOCITY.in(MetersPerSecond),
                0
            ),
            -angularVelocity,
            false
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(
            new Translation2d(0, 0),
            0, false
        );
    }

    @Override
    public boolean isFinished() {
        return m_count > 100;
    }
}
