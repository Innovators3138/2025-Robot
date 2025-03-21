package frc.robot.commands.ClimbComands;

import frc.robot.subsystems.climb.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class StopClimbing extends Command {
private final ClimbSubsystem m_climbSubsystem;
public  StopClimbing(ClimbSubsystem ClimbSubsystem) {
   this.m_climbSubsystem = ClimbSubsystem;
}

public void initialize() {
    //m_climbSubsystem.StopMotor();
  }
  public void end(boolean interrupted) {
   
  }
  public boolean isFinished() {
    
        return false;
  }

public void execute() {
  }
public void end() {
  
}
}

