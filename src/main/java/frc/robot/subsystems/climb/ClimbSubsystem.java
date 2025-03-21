package frc.robot.subsystems.climb;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.sensors.CANdiEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.hardware.CANdi;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ClimbSubsystem extends SubsystemBase {

    private SparkMax m_climbMotorController = new SparkMax(23, MotorType.kBrushless);
    private ClimberSpark m_climbMotor;
    private SparkBaseConfig m_climbConfig = new SparkMaxConfig();
    private boolean m_Activated = false;
    private final CANdiEncoder m_climberEncoder;
    private final PWM1Configs m_climberPWMConfig = new PWM1Configs();

    private final DoublePublisher m_positionPublisher;
    private final DoublePublisher m_velocityPublisher;
    private final DoublePublisher m_voltagePublisher;


    public ClimbSubsystem(CANdi candi) {
        
        m_climbMotor = new ClimberSpark(m_climbMotorController, m_climbConfig, DCMotor.getNeo550(1));

        m_climberPWMConfig.withAbsoluteSensorDiscontinuityPoint(ClimberConstants.ABSOLUTE_SENSOR_DISCONTINUITY_POINT);
        m_climberPWMConfig.withAbsoluteSensorOffset(ClimberConstants.ABSOLUTE_SENSOR_OFFSET);
        m_climberPWMConfig.withSensorDirection(ClimberConstants.ENCODER_IS_INVERTED);
        candi.getConfigurator().apply(m_climberPWMConfig);
        
        
        
        m_climberEncoder = new CANdiEncoder(candi, 2);

        m_climbMotor.setVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_climbMotor.setCurrentLimit(ArmConstants.SHOULDER_MOTOR_CURRENT_LIMIT);
        m_climbMotor.setLoopRampRate(ArmConstants.SHOULDER_MOTOR_RAMP_RATE);
        m_climbMotor.setInverted(ArmConstants.SHOULDER_MOTOR_IS_INVERTED);
        m_climbMotor.setMotorBrake(true);
        m_climbMotor.burnFlash();

        m_positionPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("climber/Raw Absolute Encoder Position").publish();
        m_velocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("climber/Raw Absolute Encoder Velocity").publish();
        m_voltagePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("climber/Voltage").publish();
        
    }

    public void activateClimbUp(double speed)
    {
        m_climbMotor.set(speed * 1.0);
    }

    public void activateClimbDown(double speed)
    {
        m_climbMotor.set(speed * -1.0);
    }
    public void stopClimb()
    {    
        m_climbMotor.set(0.0);
    }

    public AngularVelocity getClimbVelocity()
    {
        return m_climberEncoder.getVelocity();
    }

    public Angle getClimberPosition()
    {
        return m_climberEncoder.getPosition();
    }

    public void ActivateClimber()
    {
        m_Activated = true;
    }
    public boolean IsActivated()
    {
        return m_Activated;
    }

    public void updateTelemetry()
    {
        m_positionPublisher.set(getClimberPosition().in(Rotations));
        m_velocityPublisher.set(getClimbVelocity().in(RotationsPerSecond));
        m_voltagePublisher.set(m_climbMotor.getVoltage());
    }

    public void periodic()
    {
        updateTelemetry();
    }
}
