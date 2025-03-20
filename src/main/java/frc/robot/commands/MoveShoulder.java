package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.arm.Arm;



public class MoveShoulder extends Command {
    private final Arm m_armSubsystem;
    private Angle shoulderSetpoint;
    boolean finished;

    public MoveShoulder(Arm armSubsystem, Angle shoulderSetpoint) {
        m_armSubsystem = armSubsystem;
        this.shoulderSetpoint = shoulderSetpoint;
        addRequirements(m_armSubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.setShoulderPosition(shoulderSetpoint);

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
