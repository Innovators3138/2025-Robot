package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import static edu.wpi.first.units.Units.*;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class SetToLevelFour extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final Arm m_arm;
    private boolean m_OKToMove = false;
    private final GripperSubsystem m_gripperSubsystem;
    private boolean m_finishAfterSetpointReached = false;
    private boolean m_finished = false;

    public SetToLevelFour(ElevatorSubsystem elevatorSubsystem, Arm arm, GripperSubsystem gripperSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_gripperSubsystem = gripperSubsystem;
        m_arm = arm;
        addRequirements(m_elevatorSubsystem, m_arm);
    }
    public SetToLevelFour(ElevatorSubsystem elevatorSubsystem, Arm arm, GripperSubsystem gripperSubsystem, Boolean finishAfterSetpointReached) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_gripperSubsystem = gripperSubsystem;
        m_arm = arm;
        addRequirements(m_elevatorSubsystem, m_arm);
        m_finishAfterSetpointReached = finishAfterSetpointReached;
    }

    @Override
    public void initialize() {
        m_arm.stopAllMotionAndClearPIDInfo();
        m_elevatorSubsystem.stopAllMotionAndClearPIDInfo();
        m_OKToMove = false;
        m_gripperSubsystem.invertGripper();
        m_finished = false;
    }

    @Override
    public void execute() {
        if (m_OKToMove) //only perform motion operations if the shoulder and wrist are idle first to avoid crazy swing operations from a prior command
        {
            m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_L4_HEIGHT);
            m_arm.setShoulderPosition(ArmConstants.ARM_L4_ANGLES[0]);
            m_arm.setWristPosition(ArmConstants.ARM_L4_ANGLES[1]);

            if (m_finishAfterSetpointReached && m_arm.shoulderAtSetPoint() && m_arm.wristAtSetPoint() && m_elevatorSubsystem.atSetpoint())
            {
                m_finished = true;
            }
        }
        else 
        {
            final double shoulderRotationsPerSec = m_arm.getShoulderVelocity().in(RotationsPerSecond);
            final double wristRotationsPerSec = m_arm.getWristVelocity().in(RotationsPerSecond);
            if ((shoulderRotationsPerSec <= ArmConstants.SHOULDER_IDLE_SPEED_REQUIRED.in(RotationsPerSecond)) && 
                (wristRotationsPerSec <= ArmConstants.WRIST_IDLE_SPEED_REQUIRED.in(RotationsPerSecond)))
            {
                m_OKToMove = true;
            }
            else{
                System.out.println("Waiting for wrist and/or shoulder to stop.  Shoulder velocity: " + shoulderRotationsPerSec + ", wrist velocity: " + wristRotationsPerSec);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stopAllMotionAndClearPIDInfo();
        //Note: do not stop elevator.  It will maintain itself
        m_gripperSubsystem.resetGripper();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
