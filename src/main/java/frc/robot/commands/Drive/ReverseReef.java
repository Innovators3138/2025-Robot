package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ReverseReef extends Command {
    private final LinearVelocity REVERSE_SPEED = MetersPerSecond.of(0.25);
    private final SwerveSubsystem m_swerveSubsystem;
    private Pose2d m_backPose;
    private int m_count;
    public ReverseReef(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_count = 0;
        var robotPose = m_swerveSubsystem.getPose();

        m_backPose = robotPose
            .transformBy(
                new Transform2d(
                    new Translation2d(
                        Meter.of(-1).in(Meter),
                        0
                    ),
                    Rotation2d.kZero
                )
            );
    }

    @Override
    public void execute() {
        m_count++;
        if (m_backPose == null) {
            return;
        }

        m_swerveSubsystem.drive(
            new Translation2d(-REVERSE_SPEED.in(MetersPerSecond), 0),
            0, false
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(
            new Translation2d(0, 0),
            0, false);
        if (m_backPose == null) {
            return;
        }

        m_backPose = null;
    }

    @Override
    public boolean isFinished() {
        if (m_count > 150) {
            return true;
        }
        if (m_backPose == null) {
            return true;
        }

        var distance = m_backPose.getTranslation().getDistance(m_swerveSubsystem.getPose().getTranslation());
        return Math.abs(distance) < 0.01;
    }
}
