package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.hardware.CANdi;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.utils.Cache;
import frc.robot.utils.PIDFConfig;
import frc.robot.math.ArmMath;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.elevator.*;


public class Arm extends SubsystemBase {
    private SparkBaseConfig wristcfg = new SparkFlexConfig();
    private SparkBaseConfig shouldercfg = new SparkMaxConfig();
    private final ArmSpark m_shoulderMotor;
    private final ArmSpark m_wristMotor;
    private final CANdi m_armCANdi;
    private  ArmEncoder m_shoulderEncoder;
    private  ArmEncoder m_wristEncoder;
    private ArmIO io;
    public final Cache<Angle> shoulderPositionCache;
    public final Cache<AngularVelocity> shoulderVelocityCache;
    public final Cache<Angle> wristPositionCache;
    public final Cache<AngularVelocity> wristVelocityCache;
    public final PIDFConfig shoulderPIDFConfig;
    public final PIDFConfig wristPIDFConfig;
    private ArmFeedforward shoulderFeedforward;
    private ArmFeedforward wristFeedforward;

    private final DoublePublisher rawShoulderPositionPublisher;
    private final DoublePublisher rawShoulderVelocityPublisher;
    private final DoublePublisher rawWristPositionPublisher;
    private final DoublePublisher rawWristVelocityPublisher;

    public Arm() {


        m_shoulderMotor = new ArmSpark(new SparkFlex(18, MotorType.kBrushless), shouldercfg, DCMotor.getNeoVortex(1));
        m_wristMotor = new ArmSpark(new SparkMax(19, MotorType.kBrushless), wristcfg, DCMotor.getNEO(1));
        m_armCANdi = new CANdi(34);
        m_armCANdi.clearStickyFaults();
        m_shoulderEncoder = new ArmEncoder(m_armCANdi, ArmConstants.SHOULDER_ENCODER_SIGNAL); 
        m_wristEncoder = new ArmEncoder(m_armCANdi, ArmConstants.WRIST_ENCODER_SIGNAL);
        m_shoulderEncoder.configCandi();
        m_wristEncoder.configCandi();

        shoulderPIDFConfig = new PIDFConfig(ArmConstants.SHOULDER_P,
                                            ArmConstants.SHOULDER_I,
                                            ArmConstants.SHOULDER_D,
                                            ArmConstants.SHOULDER_FF,
                                            ArmConstants.SHOULDER_IZ
                                            );
        wristPIDFConfig = new PIDFConfig(ArmConstants.WRIST_P,
                                            ArmConstants.WRIST_I,
                                            ArmConstants.WRIST_D,
                                            ArmConstants.WRIST_FF,
                                            ArmConstants.WRIST_IZ
                                            );

        shoulderFeedforward = getDefaultShoulderFeedForward();
        wristFeedforward = getDefaultWristFeedForward();

        m_shoulderMotor.setVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_shoulderMotor.setCurrentLimit(ArmConstants.SHOULDER_MOTOR_CURRENT_LIMIT);
        m_shoulderMotor.setLoopRampRate(ArmConstants.SHOULDER_MOTOR_RAMP_RATE);
        m_shoulderMotor.setInverted(ArmConstants.SHOULDER_MOTOR_IS_INVERTED);
        m_shoulderMotor.setMotorBrake(true);

        m_wristMotor.setVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_wristMotor.setCurrentLimit(ArmConstants.WRIST_MOTOR_CURRENT_LIMIT);
        m_wristMotor.setLoopRampRate(ArmConstants.WRIST_MOTOR_RAMP_RATE);
        m_wristMotor.setInverted(ArmConstants.WRIST_MOTOR_IS_INVERTED);
        m_wristMotor.setMotorBrake(true);

        m_shoulderMotor.configurePIDF(shoulderPIDFConfig);
        m_wristMotor.configurePIDF(wristPIDFConfig);
        m_shoulderMotor.configurePIDWrapping(0, 360);
        m_wristMotor.configurePIDWrapping(0, 360);

        m_shoulderMotor.burnFlash();
        m_wristMotor.burnFlash();

        shoulderPositionCache = new Cache<>(m_shoulderEncoder::getPosition, 20);
        shoulderVelocityCache = new Cache<>(m_shoulderEncoder::getVelocity, 20);
        wristPositionCache = new Cache<>(m_wristEncoder::getPosition, 20);
        wristVelocityCache = new Cache<>(m_wristEncoder::getVelocity, 20);

        shoulderPositionCache.update();
        shoulderVelocityCache.update();
        wristPositionCache.update();
        wristVelocityCache.update();

        rawShoulderPositionPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Absolute Encoder Position").publish();
        rawShoulderVelocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Absolute Encoder Velocity").publish();
        rawWristPositionPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Absolute Encoder Position").publish();
        rawWristVelocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Absolute Encoder Velocity").publish();
    
        

    }

    public void updateTelemetry()
    {
        rawShoulderPositionPublisher.set(m_shoulderEncoder.getPosition().in(Degrees));
        rawShoulderVelocityPublisher.set(m_shoulderEncoder.getVelocity().in(DegreesPerSecond));
        rawWristPositionPublisher.set(m_wristEncoder.getPosition().in(Degrees));
        rawWristVelocityPublisher.set(m_wristEncoder.getVelocity().in(DegreesPerSecond));
    }

    public ArmFeedforward getDefaultShoulderFeedForward() {
        return ArmMath.createShoulderFeedforward();
    }
    public ArmFeedforward getDefaultWristFeedForward() {
        return ArmMath.createWristFeedforward();
    }

    public void periodic() {
        updateTelemetry();

    }
}
