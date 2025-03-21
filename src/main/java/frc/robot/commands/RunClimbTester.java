package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.GripperConstants;


public class RunClimbTester extends Command {
    private final ClimbSubsystem m_climbSubsystem;
    private CommandXboxController m_controller;

    public RunClimbTester(ClimbSubsystem climbSubsystem, CommandXboxController controller) {
        m_climbSubsystem = climbSubsystem;
        m_controller = controller;
        addRequirements(m_climbSubsystem);
    }
    @Override
    public void initialize() 
    {
        m_climbSubsystem.bypassPIDAndSetSpeedDirectly(0.0);
    }

    @Override
    public void execute() {

        if ((m_controller.getLeftTriggerAxis() >= GripperConstants.TRIGGER_DEADZONE) && 
            (m_controller.getRightTriggerAxis() < GripperConstants.TRIGGER_DEADZONE))
        { 
            //Use left trigger
            m_climbSubsystem.bypassPIDAndSetSpeedDirectly(GripperConstants.GRIPPER_VOLTAGE_COEFFICIENT * m_controller.getLeftTriggerAxis());
        }
        else if ((m_controller.getLeftTriggerAxis() < GripperConstants.TRIGGER_DEADZONE) && 
                 (m_controller.getRightTriggerAxis() >= GripperConstants.TRIGGER_DEADZONE))
        {
            //use right trigger
            m_climbSubsystem.bypassPIDAndSetSpeedDirectly(GripperConstants.GRIPPER_VOLTAGE_COEFFICIENT * m_controller.getRightTriggerAxis() * -1);
        }
        else
        {
            m_climbSubsystem.bypassPIDAndSetSpeedDirectly(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) 
    {
        m_climbSubsystem.bypassPIDAndSetSpeedDirectly(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; //run forever unless interrupted
    }
}
