package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class BumpReef extends Command {
    private final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(10);
    private final LinearVelocity BUMP_VELOCITY = MetersPerSecond.of(0.25);
    private final SwerveSubsystem m_swerveSubsystem;
    private final PIDController m_headingController = new PIDController(0.2, 0, 0);
    private int m_count;
    private double m_originalHeading;

    private final DoublePublisher m_driftPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/drift")
        .publish();

    private final DoublePublisher m_driftCorrectionPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("vision/align/driftCorrection")
        .publish();

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

        m_driftPublisher.set(headingDrift);

        var angularVelocity = -MathUtil.clamp(
            m_headingController.calculate(headingDrift),
            -MAX_ANGULAR_VELOCITY.in(DegreesPerSecond),
            MAX_ANGULAR_VELOCITY.in(DegreesPerSecond));

        m_driftCorrectionPublisher.set(angularVelocity);

        // TODO for now I am not trying to correct the heading using the calculated angular velocity
        // in testing, I am not sure it helped much.  ATM we enabled heading correcting in the swerve
        // drive.
        m_swerveSubsystem.drive(
            new Translation2d(
                BUMP_VELOCITY.in(MetersPerSecond),
                0
            ),
            0,
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
        return m_count > 150;
    }
}
