package frc.robot.subsystems.climb;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.sensors.CANdiEncoder;
import frc.robot.utils.PIDFConfig;

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
    private PIDController m_pidController;
    private Servo m_ratchetServo;

    private final DoublePublisher m_positionPublisher;
    private final DoublePublisher m_velocityPublisher;
    private final DoublePublisher m_voltagePublisher;
    private final DoublePublisher m_pidSetPointPublisher;


    public ClimbSubsystem(CANdi candi) {
        
        m_ratchetServo = new Servo(ClimberConstants.RATCHET_SERVO_CHAN_NUM);
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


        m_pidController = new PIDController(ClimberConstants.CLIMBER_P, ClimberConstants.CLIMBER_I, ClimberConstants.CLIMBER_D);
        m_pidController.setTolerance(ClimberConstants.PID_TOLERANCE_ROTATIONS);
        m_pidController.setIZone(ClimberConstants.PID_IZONE_ROTATIONS);
        m_pidController.setIntegratorRange(ClimberConstants.PID_MAX_INTEGRATOR, ClimberConstants.PID_MIN_INTEGRATOR);

        m_positionPublisher = NetworkTableInstance.getDefault().getTable("AdvantageKit").getDoubleTopic("climber/Raw Absolute Encoder Position").publish();
        m_velocityPublisher = NetworkTableInstance.getDefault().getTable("AdvantageKit").getDoubleTopic("climber/Raw Absolute Encoder Velocity").publish();
        m_voltagePublisher = NetworkTableInstance.getDefault().getTable("AdvantageKit").getDoubleTopic("climber/Voltage").publish();
        m_pidSetPointPublisher = NetworkTableInstance.getDefault().getTable("AdvantageKit").getDoubleTopic("climber/PID Set Point Position").publish();
    }

    public void stopAllMotionAndClearPIDInfo()
    {
        m_pidController.reset();
        m_climbMotor.setVoltage(0);
    }

    public void updatePIDToMaintainSetPoint()
    {
        double voltage = m_pidController.calculate(getClimberPosition().in(Rotations));

        if (Constants.ClimberConstants.MOTOR_IS_INVERTED)
        {
            voltage *= -1;
        }

        //enforce max voltage allowed for the motor
        voltage = Math.max(Constants.NOMINAL_VOLTAGE, voltage);
        voltage = Math.min(Constants.NOMINAL_VOLTAGE, voltage);

        m_climbMotor.setVoltage(voltage);
    }

    public void bypassPIDAndSetSpeedDirectly(double speed)
    {
        m_pidController.reset();

        double voltage = speed * Constants.NOMINAL_VOLTAGE;
        voltage = Math.max(Constants.NOMINAL_VOLTAGE, voltage);
        voltage = Math.min(Constants.NOMINAL_VOLTAGE, voltage);

        m_climbMotor.setVoltage(speed * Constants.NOMINAL_VOLTAGE);
    }

    public void setClimberPositionAndUpdatePID(Angle setPoint)
    {
        if ((setPoint.in(Rotations) < ClimberConstants.MAX_ANGLE.in(Rotations)) && 
            (setPoint.in(Rotations) > ClimberConstants.MIN_ANGLE.in(Rotations)))
        {
            m_pidController.setSetpoint(setPoint.in(Rotations));
        }
        else
        {
            System.out.println("Illegal climber set point provided: " + setPoint.in(Rotations));
        }

        updatePIDToMaintainSetPoint();
    }

    public boolean atSetPoint() 
    { 
        return m_pidController.atSetpoint(); 
    }

    public void enableRatchet() 
    {
        m_ratchetServo.set(0.1);
    }
    public void disableRatchet()
    {
        m_ratchetServo.set(0.0);
    }


    public AngularVelocity getClimbVelocity()
    {
        return m_climberEncoder.getVelocity();
    }

    public Angle getClimberPosition()
    {
        return m_climberEncoder.getPosition();
    }

    public void markClimberActivated()
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
        m_pidSetPointPublisher.set(m_pidController.getSetpoint());
    }

    public void periodic()
    {
        updateTelemetry();
    }
}
