package frc.robot.subsystems.climb;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class ClimbSubsystem extends SubsystemBase {
        private final SparkMax m_climbMotorController = new SparkMax(23, MotorType.kBrushless);
        private boolean m_Activated = false;

    public ClimbSubsystem() {
        m_climbMotorController.set(0.0);
    }

    public void Climb() {
        m_climbMotorController.set(1.0);
    }
    public void StopMotor(){    
    m_climbMotorController.set(0.0);
    }
    public void ActivateClimber(){
    m_Activated = true;
    }
    public boolean IsActivated(){
        return m_Activated;
    }
}
