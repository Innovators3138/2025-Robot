package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.arm.Arm;



public class MoveWrist extends Command {
    private final Arm m_armSubsystem;
    private Angle wristSetpoint;
    boolean finished;

    public MoveWrist(Arm armSubsystem, Angle wristSetpoint) {
        m_armSubsystem = armSubsystem;
        this.wristSetpoint = wristSetpoint;
        addRequirements(m_armSubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.setWristPosition(wristSetpoint);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
