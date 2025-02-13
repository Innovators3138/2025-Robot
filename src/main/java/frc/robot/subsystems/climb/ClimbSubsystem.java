package frc.robot.subsystems.climb;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax m_climbMotorController = new SparkMax(1, MotorType.kBrushless);
  

    public ClimbSubsystem() {
        m_climbMotorController.set(0.0);
    }

    public void Climb() {
        m_climbMotorController.set(1.0);
        try {
            wait(100);
        }
        catch (InterruptedException e) {
            //do nothing
        }
        m_climbMotorController.set(0.0);
    }
}
