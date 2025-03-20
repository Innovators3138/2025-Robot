package frc.robot.commands.ClimbComands;
import frc.robot.subsystems.climb.ClimbSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class ClimbCommand extends Command {
private int m_climbCountdown = 0;
private CommandXboxController m_driveController;
private CommandXboxController m_operatorController;  
private final ClimbSubsystem m_climbSubsystem;

public  ClimbCommand(ClimbSubsystem ClimbSubsystem, CommandXboxController DriveController, CommandXboxController OperatorController) {
   this.m_climbSubsystem = ClimbSubsystem;
   m_driveController = DriveController;
   m_operatorController = OperatorController;
   addRequirements(m_climbSubsystem);
}



public void initialize() {
    m_climbCountdown = 150;
  }

   
  
  public boolean isFinished() {
    
        return false;
  }

public void execute() {
if (m_climbSubsystem.IsActivated() == false){
    
if ((m_driveController.getLeftTriggerAxis() == 1.0) && (m_driveController.getRightTriggerAxis() == 1.0)){
    if (m_climbCountdown == 0){
  m_climbSubsystem.ActivateClimber();
    System.out.println("Climber Activated");
  }else {
    m_climbCountdown--; 
  }
  }else{
    m_climbCountdown = 150;
  }
  }
  }
public void end() {
  m_climbSubsystem.StopMotor();
}
}


