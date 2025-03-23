package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.GripperConstants;

public class RunGripper extends Command {
    private final GripperSubsystem m_GripperSubsystem;
    private CommandXboxController m_controller;

    public RunGripper(GripperSubsystem gripperSubsystem, CommandXboxController controller) {
        m_GripperSubsystem = gripperSubsystem;
        m_controller = controller;
        addRequirements(m_GripperSubsystem);
    }

    @Override
    public void initialize() {
        m_GripperSubsystem.setIntakeSpeed(0.0);
    }

    @Override
    public void execute() {

        if ((m_controller.getLeftTriggerAxis() >= GripperConstants.TRIGGER_DEADZONE) &&
                (m_controller.getRightTriggerAxis() < GripperConstants.TRIGGER_DEADZONE)) {
            if (m_controller.getLeftTriggerAxis() >= 0.95) {
                m_GripperSubsystem.setIntakeSpeed(GripperConstants.GRIPPER_MAX_SPEED);
            } else {

                // Use left trigger
                m_GripperSubsystem.setIntakeSpeed(
                        GripperConstants.GRIPPER_VOLTAGE_COEFFICIENT * m_controller.getLeftTriggerAxis());
            }
        } else if ((m_controller.getLeftTriggerAxis() < GripperConstants.TRIGGER_DEADZONE) &&
                (m_controller.getRightTriggerAxis() >= GripperConstants.TRIGGER_DEADZONE)) {
            if (m_controller.getRightTriggerAxis() >= 0.95) {
                m_GripperSubsystem.setIntakeSpeed(GripperConstants.GRIPPER_MAX_SPEED);
            } else {

                // use right trigger
                m_GripperSubsystem.setIntakeSpeed(
                        (GripperConstants.GRIPPER_VOLTAGE_COEFFICIENT * m_controller.getRightTriggerAxis()) * -1);
            }
        } else {
            m_GripperSubsystem.setIntakeSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_GripperSubsystem.setIntakeSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // run forever unless interrupted
    }
}
