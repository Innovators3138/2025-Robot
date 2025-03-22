package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;


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


public class Arm extends SubsystemBase {
    private SparkBaseConfig wristcfg = new SparkMaxConfig();
    private SparkBaseConfig shouldercfg = new SparkFlexConfig();
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
    private Angle wristSetpoint = Rotations.of(0);
    private Angle shoulderSetpoint = Rotations.of(0);
    private Angle wristError = Rotations.of(0);
    private Angle shoulderError = Rotations.of(0);

    private final DoublePublisher rawShoulderPositionPublisher;
    private final DoublePublisher rawShoulderVelocityPublisher;
    private final DoublePublisher rawShoulderSetpointPublisher;
    private final DoublePublisher rawShoulderErrorPublisher;
    private final DoublePublisher rawShoulderDerivativePublisher;
    private final DoublePublisher rawShoulderVoltagePublisher;
    private final DoublePublisher rawShoulderFeedforwardPublisher;
    private final DoublePublisher shoulderErrorAccumulation;
    private final DoublePublisher rawWristPositionPublisher;
    private final DoublePublisher rawWristVelocityPublisher;
    private final DoublePublisher rawWristSetpointPublisher;
    private final DoublePublisher rawWristErrorPublisher;
    private final DoublePublisher rawWristVoltagePublisher;
    public boolean wristAtSetpoint = false;
    private double shoulderFeedforwardVoltage = 0.0;
    private double shoulderAccumulatedError = 0.0;
    private double m_shoulderErrorDerivative = 0.0;
    private final PIDController wristController;
    private final PIDController shoulderController;

    public Arm(CANdi armCANDi) {


        m_shoulderMotor = new ArmSpark(new SparkFlex(18, MotorType.kBrushless), shouldercfg, DCMotor.getNeoVortex(1));
        m_wristMotor = new ArmSpark(new SparkMax(19, MotorType.kBrushless), wristcfg, DCMotor.getNeo550(1));
        m_armCANdi = armCANDi;
        m_armCANdi.clearStickyFaults();
        
        m_shoulderEncoder = new ArmEncoder(m_armCANdi, ArmConstants.SHOULDER_ENCODER_SIGNAL); 
        m_wristEncoder = new ArmEncoder(m_armCANdi, ArmConstants.WRIST_ENCODER_SIGNAL);
        m_shoulderEncoder.configCandi();
        m_wristEncoder.configCandi();
        shoulderFeedforward = ArmMath.createShoulderFeedforward();
        
        

        

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
        rawShoulderSetpointPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Position Setpoint").publish();
        rawShoulderErrorPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Position Error").publish();
        rawShoulderDerivativePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw position derivative error").publish();
        rawShoulderVoltagePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Voltage").publish();
        rawShoulderFeedforwardPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Feedforward Voltage").publish();
        rawWristPositionPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Absolute Encoder Position").publish();
        rawWristVelocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Absolute Encoder Velocity").publish();
        rawWristSetpointPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Position Setpoint").publish();
        rawWristErrorPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Position Error").publish();
        rawWristVoltagePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Voltage").publish();
        shoulderErrorAccumulation = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Shoulder Accumulated Error").publish();

        shoulderFeedforwardVoltage = shoulderFeedforward.calculate((getShoulderPosition()).in(Radians), getShoulderVelocity().in(RadiansPerSecond));
        
        wristController = new PIDController(ArmConstants.WRIST_P, ArmConstants.WRIST_I, ArmConstants.WRIST_D);
        wristController.setTolerance(0.005);
        wristController.setIZone(ArmConstants.WRIST_IZ);
        shoulderController = new PIDController(ArmConstants.SHOULDER_P, ArmConstants.SHOULDER_I, ArmConstants.SHOULDER_D);
        shoulderController.setTolerance(0.005);
        shoulderController.setIZone(ArmConstants.SHOULDER_IZ);

    }
    public Angle getWristPosition() {
        double position = m_wristEncoder.getPosition().in(Rotations);
        return Rotations.of(position);
    }

    public AngularVelocity getWristVelocity() {
        return RotationsPerSecond.of(m_wristEncoder.getVelocity().in(RotationsPerSecond));
    }

    public void setWristPosition(Angle setpoint) {
        Angle position = getWristPosition();

        if (wristSetpoint != setpoint)
        {
            System.out.println("Changing wrist setpoint to " + setpoint.in(Rotations));
            this.wristSetpoint = setpoint;
        }
        this.wristError = position.minus(setpoint);

        
        if (wristController.atSetpoint()) {
            wristAtSetpoint = true;
        } else {
            wristAtSetpoint = false;
        }
        double voltage = wristController.calculate(getWristPosition().in(Rotations), setpoint.in(Rotations));
        if (Constants.ArmConstants.WRIST_MOTOR_IS_INVERTED) {
            voltage = -1 * voltage;
        }
 
        m_wristMotor.setVoltage(voltage);
    }

    public boolean wristAtSetPoint() { return wristController.atSetpoint(); }
    public boolean shoulderAtSetPoint() { return shoulderController.atSetpoint(); }

    public Angle getShoulderPosition() {
        double position = m_shoulderEncoder.getPosition().in(Rotations) * ArmConstants.SHOULDER_CONVERSION_FACTOR;
        return Rotations.of(position);
    }

    public AngularVelocity getShoulderVelocity() {
        return RotationsPerSecond.of(m_shoulderEncoder.getVelocity().in(RotationsPerSecond) * ArmConstants.SHOULDER_CONVERSION_FACTOR);
    }

    public void stopAllMotionAndClearPIDInfo()
    {
        shoulderController.reset();
        wristController.reset();
        m_shoulderMotor.setVoltage(0);
        m_wristMotor.setVoltage(0);
    }

    public void setShoulderPosition(Angle setpoint) {
        Angle position = getShoulderPosition();
        shoulderFeedforwardVoltage = shoulderFeedforward.calculate((position).in(Radians) + Math.PI / 2, getShoulderVelocity().in(RadiansPerSecond));
        this.shoulderError = position.minus(setpoint);  //TODO should we just use shoulderController.getError()?

        double voltage = 0.0;

        //add protection for arm moving out of bounds
        if (position.in(Rotations) < ArmConstants.SHOULDER_MIN_SAFE_ANGLE.in(Rotations))
        {
            shoulderController.reset();

            voltage = 1.0; //force arm to move back into bounds until its in valid range for PID again

            System.out.println("Out of bounds shoulder position detected.  Setting voltage to 1.0 to force back into position");
        }
        else if (position.in(Rotations) > ArmConstants.SHOULDER_MAX_SAFE_ANGLE.in(Rotations))
        {
            shoulderController.reset();

            voltage = -1.0; //force arm to move back into bounds until its in valid range for PID again

            System.out.println("Out of bounds shoulder position detected.  Setting voltage to -1.0 to force back into position");
        }
        else
        {

            if (setpoint != this.shoulderSetpoint)
            {
                this.shoulderSetpoint = setpoint; 
            
                System.out.println("Setting shoulder position setpoint " + setpoint + ", shoulder feed forward voltage: " + shoulderFeedforwardVoltage + ", current shoulder position is " + getShoulderPosition().in(Rotations));
            }

            m_shoulderErrorDerivative = shoulderController.getErrorDerivative();
            shoulderAccumulatedError = shoulderController.getAccumulatedError();
            shoulderController.getError();
            voltage = shoulderController.calculate(getShoulderPosition().in(Rotations), setpoint.in(Rotations));

            if (Constants.ArmConstants.SHOULDER_MOTOR_IS_INVERTED) {
                voltage = -1 * voltage;
            }

            if (shoulderController.atSetpoint())
            {
                voltage = 0;
            }
        }


        if (m_shoulderMotor.getVoltage() != voltage)
        {
            //System.out.println("setting shoulder voltage to " + voltage);
            //System.out.println("calculated voltage before adding is " + voltage + ", at setpoint: " + shoulderController.atSetpoint());

            //voltage = voltage + Math.abs(shoulderFeedforwardVoltage) * Math.signum(voltage);  //TODO disable feed forward for now until we get things stable

            m_shoulderMotor.setVoltage(voltage);
        }
        
    }

    public void updateTelemetry()
    {
        rawShoulderPositionPublisher.set(getShoulderPosition().in(Rotations));
        rawShoulderVelocityPublisher.set(getShoulderVelocity().in(RotationsPerSecond));
        rawShoulderSetpointPublisher.set(shoulderSetpoint.in(Rotations));
        rawShoulderErrorPublisher.set(shoulderError.in(Rotations));
        rawShoulderDerivativePublisher.set(m_shoulderErrorDerivative);
        rawShoulderVoltagePublisher.set(m_shoulderMotor.getVoltage());
        rawShoulderFeedforwardPublisher.set(shoulderFeedforwardVoltage);        
        rawWristPositionPublisher.set(m_wristEncoder.getPosition().in(Rotations));
        rawWristVelocityPublisher.set(m_wristEncoder.getVelocity().in(RotationsPerSecond));
        rawWristSetpointPublisher.set(wristSetpoint.in(Rotations));
        rawWristErrorPublisher.set(wristError.in(Rotations));
        rawWristVoltagePublisher.set(m_wristMotor.getLastSetVoltage());
        shoulderErrorAccumulation.set(shoulderAccumulatedError);
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
