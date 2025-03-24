package frc.robot.commands.ClimbComands;

import frc.robot.Constants.*;
import frc.robot.subsystems.climb.ClimbSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.units.Units.*;

public class ClimbCommand extends Command {
    private int m_activateClimberCountdown = 0;
    private CommandXboxController m_driveController;
    private CommandXboxController m_operatorController;
    private final ClimbSubsystem m_climbSubsystem;
    private boolean m_climberActivationInProgress = false;
    private boolean m_ratchetEnabled = false;

    

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
            if (m_operatorController.leftBumper().getAsBoolean() && 
                m_operatorController.rightBumper().getAsBoolean()) 
            {
                if (m_activateClimberCountdown == 0) 
                {
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
            // if ((m_operatorController.getLeftTriggerAxis() >= ClimberConstants.TRIGGER_ACTIVATION_LEVEL) && 
            //     (m_operatorController.getRightTriggerAxis() < ClimberConstants.TRIGGER_ACTIVATION_LEVEL))
            // { 
            //     //Use left trigger
            //     m_climbSubsystem.setClimberPositionAndUpdatePID(ClimberConstants.STOW_ANGLE);
            // }
            // else if ((m_operatorController.getLeftTriggerAxis() < ClimberConstants.TRIGGER_ACTIVATION_LEVEL) && 
            //         (m_operatorController.getRightTriggerAxis() >= ClimberConstants.TRIGGER_ACTIVATION_LEVEL))
            // {
            //     //use right trigger
            //     m_climbSubsystem.setClimberPositionAndUpdatePID(ClimberConstants.CLIMB_ANGLE);
            // }
            // else
            // {
            //     m_climbSubsystem.updatePIDToMaintainSetPoint();
            // }

            if ((m_operatorController.getLeftTriggerAxis() >= GripperConstants.TRIGGER_DEADZONE) &&  //button to raise robot using climber
                (m_operatorController.getRightTriggerAxis() < GripperConstants.TRIGGER_DEADZONE))
            { 
                if (!m_ratchetEnabled) //turn on ratchet when climbing
                {
                    m_climbSubsystem.enableRatchet();
                    m_ratchetEnabled = true;
                }

                if (m_climbSubsystem.getClimberPosition().in(Rotations) <= ClimberConstants.CLIMB_ANGLE.in(Rotations)) //TODO < > orientation on this
                {
                    m_climbSubsystem.bypassPIDAndSetSpeedDirectly(0.5 * Math.pow(m_operatorController.getLeftTriggerAxis(), 2.0));
                }
            }
            else if ((m_operatorController.getLeftTriggerAxis() < ClimberConstants.TRIGGER_ACTIVATION_LEVEL) &&  //button to lower robot using climber
                    (m_operatorController.getRightTriggerAxis() >= ClimberConstants.TRIGGER_ACTIVATION_LEVEL))
            {
                if (m_ratchetEnabled)  //remove ratchet to descend
                {
                    m_climbSubsystem.disableRatchet();
                    m_ratchetEnabled = false;
                }

                if (m_climbSubsystem.getClimberPosition().in(Rotations) >= ClimberConstants.STOW_ANGLE.in(Rotations))  //TODO < > orientation on this
                {
                    m_climbSubsystem.bypassPIDAndSetSpeedDirectly(0.5 * Math.pow(m_operatorController.getLeftTriggerAxis(), 2.0));
                }
            }

            
        }
    }

    public void end() {
        m_climbSubsystem.stopAllMotionAndClearPIDInfo();
    }
}
