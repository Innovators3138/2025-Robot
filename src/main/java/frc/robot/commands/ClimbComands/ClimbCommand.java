package frc.robot.commands.ClimbComands;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ClimbCommand extends Command {
    private int m_activateClimberCountdown = 0;
    private CommandXboxController m_driveController;
    private CommandXboxController m_operatorController;
    private final ClimbSubsystem m_climbSubsystem;
    private boolean m_climberActivationInProgress = false;

    

    public ClimbCommand(ClimbSubsystem ClimbSubsystem, CommandXboxController DriveController,
            CommandXboxController OperatorController) {
        this.m_climbSubsystem = ClimbSubsystem;
        m_driveController = DriveController;
        m_operatorController = OperatorController;
        addRequirements(m_climbSubsystem);
    }

    public void initialize() {
        m_climbSubsystem.stopAllMotionAndClearPIDInfo();
        m_activateClimberCountdown = 150;
        m_climberActivationInProgress = false;
    }

    public boolean isFinished() {
        return false;  //run forever
    }

    public void execute() {
        if (m_climberActivationInProgress)
        {
            m_climbSubsystem.setClimberPositionAndUpdatePID(ClimberConstants.RELEASE_ANGLE);
            if (m_climbSubsystem.atSetPoint())
            {
                m_climbSubsystem.stopAllMotionAndClearPIDInfo();
                m_climbSubsystem.markClimberActivated();
                m_climberActivationInProgress = false;
            }
        }
        else if (!m_climbSubsystem.IsActivated())
        {
            if ((m_driveController.getLeftTriggerAxis() >= ClimberConstants.TRIGGER_ACTIVATION_LEVEL) && 
                (m_driveController.getRightTriggerAxis() >= ClimberConstants.TRIGGER_ACTIVATION_LEVEL)) {
                if (m_activateClimberCountdown == 0) {
                    m_climberActivationInProgress = true;
                    System.out.println("Climber Activated");
                } else {
                    m_activateClimberCountdown--;
                }
            } else { //reset count so they have to press the buttons again for the full time period
                m_activateClimberCountdown = 150;
            }
        }
        else //Climber is activated
        {
            if ((m_operatorController.getLeftTriggerAxis() >= ClimberConstants.TRIGGER_ACTIVATION_LEVEL) && 
                (m_operatorController.getRightTriggerAxis() < ClimberConstants.TRIGGER_ACTIVATION_LEVEL))
            { 
                //Use left trigger
                m_climbSubsystem.setClimberPositionAndUpdatePID(ClimberConstants.STOW_ANGLE);
            }
            else if ((m_operatorController.getLeftTriggerAxis() < ClimberConstants.TRIGGER_ACTIVATION_LEVEL) && 
                    (m_operatorController.getRightTriggerAxis() >= ClimberConstants.TRIGGER_ACTIVATION_LEVEL))
            {
                //use right trigger
                m_climbSubsystem.setClimberPositionAndUpdatePID(ClimberConstants.CLIMB_ANGLE);
            }
            else
            {
                m_climbSubsystem.updatePIDToMaintainSetPoint();
            }
        }
    }

    public void end() {
        m_climbSubsystem.stopAllMotionAndClearPIDInfo();
    }
}
