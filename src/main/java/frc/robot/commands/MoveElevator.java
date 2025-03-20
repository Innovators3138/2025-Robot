package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.elevator.ElevatorSubsystem;



public class MoveElevator extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private Distance elevatorSetpoint;
    boolean finished;

    public MoveElevator(ElevatorSubsystem elevatorSubsystem, Distance elevatorSetpoint) {
        m_elevatorSubsystem = elevatorSubsystem;
        this.elevatorSetpoint = elevatorSetpoint;
        addRequirements(m_elevatorSubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setElevatorPosition(elevatorSetpoint);
        if (m_elevatorSubsystem.atSetpoint) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
