// Author: Innovators robotics
// Last Updated : today

package frc.robot.subsystems.gripper;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {

    
    
        final private SparkMax m_feederMotor = new SparkMax(20, SparkMax.MotorType.kBrushless); 
        final private AnalogInput m_lightsensor = new AnalogInput(0);
        private final DoublePublisher rawGripperSpeed;
        private final DoublePublisher rawLightSensorVoltage;
        private final BooleanPublisher rawLightSensorTripped;
        private boolean m_isInverted = false;
        private  SparkMaxConfig   m_feederCfg   = new SparkMaxConfig();
    
        
        
        public GripperSubsystem(){
            System.out.println("Gripper subsystem constructor entered");
            //m_feederCfg.voltageCompensation(12);
            //m_feederMotor.configure(m_feederCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            m_lightsensor.setAverageBits(2);
            rawGripperSpeed = NetworkTableInstance.getDefault().getTable("AdvantageKit").getDoubleTopic("gripper/Raw Gripper Speed").publish();
            rawLightSensorVoltage = NetworkTableInstance.getDefault().getTable("AdvantageKit").getDoubleTopic("gripper/Raw Light Sensor Voltage").publish();
            rawLightSensorTripped = NetworkTableInstance.getDefault().getTable("AdvantageKit").getBooleanTopic("gripper/Light Sensor Tripped").publish();


        }

        public void setIntakeSpeed(double intakeSpeed) {
            //System.out.println("Setting intake speed to " + intakeSpeed + ", which is " + (intakeSpeed * 12) + " volts");
            if (m_isInverted == true){
                m_feederMotor.setVoltage(intakeSpeed * -12.0);

            }else{
            m_feederMotor.setVoltage(intakeSpeed * 12.0);
        }
    }
        public double getLightSensorValue() {

            return m_lightsensor.getAverageVoltage();
        }

        public void intakeIn() {
            m_feederMotor.setVoltage(5.0); 
        }

        public boolean lightSensorTripped() {
            if (getLightSensorValue() > GripperConstants.LIGHT_SENSOR_THRESHOLD) {
                return true;
            }
            else {
                return false;
            }
        }

        public void updateTelemetry()
    {
            rawGripperSpeed.set(m_feederMotor.getAppliedOutput());
            rawLightSensorVoltage.set(getLightSensorValue());
            rawLightSensorTripped.set(lightSensorTripped());
    }
    public void periodic()
    {
        updateTelemetry();
    }
    public void invertGripper(){
        m_isInverted = true;
    }
    public void resetGripper(){
        m_isInverted = false;
    }


}
