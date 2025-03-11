package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Level;

public class MoveArm extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final Arm m_arm;
    private Level m_level = Level.LEVEL_1;
    private Distance elevatorSetpoint;
    private Angle shoulderSetpoint;
    private Angle wristSetpoint;

    public MoveArm(ElevatorSubsystem elevatorSubsystem, Arm arm, Angle shoulderSetpoint, Angle wristSetpoint, Distance elevatorSetpoint) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_arm = arm;
        this.shoulderSetpoint = shoulderSetpoint;
        this.wristSetpoint = wristSetpoint;
        this.elevatorSetpoint = elevatorSetpoint;
        addRequirements(m_elevatorSubsystem, m_arm);
    }

    public MoveArm(ElevatorSubsystem elevatorSubsystem, Arm arm, Level level) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_arm = arm;
        m_level = level;
        addRequirements(m_elevatorSubsystem, m_arm);
        switch(m_level) {
            case STOWED:
                shoulderSetpoint = ArmConstants.ARM_STOWED_ANGLES[0];
                wristSetpoint = ArmConstants.ARM_STOWED_ANGLES[1];
                elevatorSetpoint = ElevatorConstants.ELEVATOR_STOWED_HEIGHT;
                break;
            case LEVEL_1:
                shoulderSetpoint = ArmConstants.ARM_L1_ANGLES[0];
                wristSetpoint = ArmConstants.ARM_L1_ANGLES[1];
                elevatorSetpoint = ElevatorConstants.ELEVATOR_L1_HEIGHT;
                break;
            case LEVEL_2:
                shoulderSetpoint = ArmConstants.ARM_L2_ANGLES[0];
                wristSetpoint = ArmConstants.ARM_L2_ANGLES[1];
                elevatorSetpoint = ElevatorConstants.ELEVATOR_L2_HEIGHT;
                break;
            case LEVEL_3:
                shoulderSetpoint = ArmConstants.ARM_L3_ANGLES[0];
                wristSetpoint = ArmConstants.ARM_L3_ANGLES[1];
                elevatorSetpoint = ElevatorConstants.ELEVATOR_L3_HEIGHT;
                break;
            case LEVEL_4:
                shoulderSetpoint = ArmConstants.ARM_L4_ANGLES[0];
                wristSetpoint = ArmConstants.ARM_L4_ANGLES[1];
                elevatorSetpoint = ElevatorConstants.ELEVATOR_L4_HEIGHT;
                break;
        }
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //Need to do all the safety things so that the arm doesn't crash into the robot if the elevator isn't up.
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
