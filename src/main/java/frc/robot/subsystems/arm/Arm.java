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
    private SparkBaseConfig m_wristcfg = new SparkMaxConfig();
    private SparkBaseConfig m_shouldercfg = new SparkFlexConfig();
    private final ArmSpark m_shoulderMotor;
    private final ArmSpark m_wristMotor;
    private final CANdi m_armCANdi;
    private  ArmEncoder m_shoulderEncoder;
    private  ArmEncoder m_wristEncoder;
    public final PIDFConfig shoulderPIDFConfig;
    public final PIDFConfig wristPIDFConfig;
    private ArmFeedforward m_shoulderFeedforward;
    private ArmFeedforward m_wristFeedforward;
    private Angle wristSetpoint = Rotations.of(0);
    private Angle m_shoulderSetpoint = Rotations.of(0);
    private Angle wristError = Rotations.of(0);
    private Angle m_calculatedShoulderError = Rotations.of(0);

    private final DoublePublisher m_rawShoulderPositionPublisher;
    private final DoublePublisher m_rawShoulderVelocityPublisher;
    private final DoublePublisher m_rawShoulderSetpointPublisher;
    private final DoublePublisher m_rawShoulderErrorPublisher;
    private final DoublePublisher m_rawShoulderDerivativePublisher;
    private final DoublePublisher m_rawShoulderVoltagePublisher;
    private final DoublePublisher m_rawShoulderFeedforwardPublisher;
    private final DoublePublisher m_shoulderErrorAccumulation;
    private final DoublePublisher m_rawWristPositionPublisher;
    private final DoublePublisher m_rawWristVelocityPublisher;
    private final DoublePublisher m_rawWristSetpointPublisher;
    private final DoublePublisher m_rawWristErrorPublisher;
    private final DoublePublisher m_rawWristVoltagePublisher;
    public boolean m_wristAtSetpoint = false;
    private double m_shoulderFeedforwardVoltage = 0.0;
    private double m_shoulderAccumulatedError = 0.0;
    private double m_shoulderErrorDerivative = 0.0;
    private final PIDController m_wristController;
    private final PIDController m_shoulderController;

    private boolean m_shoulderSetpointMoveInProgress = false;

    public Arm(CANdi armCANDi) {


        m_shoulderMotor = new ArmSpark(new SparkFlex(18, MotorType.kBrushless), m_shouldercfg, DCMotor.getNeoVortex(1));
        m_wristMotor = new ArmSpark(new SparkMax(19, MotorType.kBrushless), m_wristcfg, DCMotor.getNeo550(1));
        m_armCANdi = armCANDi;
        m_armCANdi.clearStickyFaults();
        
        m_shoulderEncoder = new ArmEncoder(m_armCANdi, ArmConstants.SHOULDER_ENCODER_SIGNAL); 
        m_wristEncoder = new ArmEncoder(m_armCANdi, ArmConstants.WRIST_ENCODER_SIGNAL);
        m_shoulderEncoder.configCandi();
        m_wristEncoder.configCandi();
        m_shoulderFeedforward = ArmMath.createShoulderFeedforward();
        
        

        

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

        m_rawShoulderPositionPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Absolute Encoder Position").publish();
        m_rawShoulderVelocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Absolute Encoder Velocity").publish();
        m_rawShoulderSetpointPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Position Setpoint").publish();
        m_rawShoulderErrorPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Position Error").publish();
        m_rawShoulderDerivativePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw position derivative error").publish();
        m_rawShoulderVoltagePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Voltage").publish();
        m_rawShoulderFeedforwardPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Raw Feedforward Voltage").publish();
        m_rawWristPositionPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Absolute Encoder Position").publish();
        m_rawWristVelocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Absolute Encoder Velocity").publish();
        m_rawWristSetpointPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Position Setpoint").publish();
        m_rawWristErrorPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Position Error").publish();
        m_rawWristVoltagePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/wrist/Raw Voltage").publish();
        m_shoulderErrorAccumulation = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("arm/shoulder/Shoulder Accumulated Error").publish();

        m_shoulderFeedforwardVoltage = m_shoulderFeedforward.calculate((getShoulderPosition()).in(Radians), getShoulderVelocity().in(RadiansPerSecond));
        
        m_wristController = new PIDController(ArmConstants.WRIST_P, ArmConstants.WRIST_I, ArmConstants.WRIST_D);
        m_wristController.setTolerance(0.005);
        m_wristController.setIZone(ArmConstants.WRIST_IZ);
        m_shoulderController = new PIDController(ArmConstants.SHOULDER_P, ArmConstants.SHOULDER_I, ArmConstants.SHOULDER_D);
        m_shoulderController.setTolerance(0.005);
        m_shoulderController.setIZone(ArmConstants.SHOULDER_IZ);

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
        this.wristSetpoint = setpoint;
        this.wristError = position.minus(setpoint);

        
        if (m_wristController.atSetpoint()) {
            m_wristAtSetpoint = true;
        } else {
            m_wristAtSetpoint = false;
        }
        double voltage = m_wristController.calculate(getWristPosition().in(Rotations), setpoint.in(Rotations));
        if (Constants.ArmConstants.WRIST_MOTOR_IS_INVERTED) {
            voltage = -1 * voltage;
        }
        System.out.println("Setting WRIST voltage to " + voltage);
        m_wristMotor.setVoltage(voltage);
    }

    public Angle getShoulderPosition() {
        double position = m_shoulderEncoder.getPosition().in(Rotations) * ArmConstants.SHOULDER_CONVERSION_FACTOR;
        return Rotations.of(position);
    }

    public AngularVelocity getShoulderVelocity() {
        return RotationsPerSecond.of(m_shoulderEncoder.getVelocity().in(RotationsPerSecond) * ArmConstants.SHOULDER_CONVERSION_FACTOR);
    }

    public void setShoulderPosition(Angle setpoint) {
        m_shoulderSetpoint = setpoint;
        m_shoulderSetpointMoveInProgress = true;

        System.out.println("Setting shoulder position setpoint " + setpoint + ", current shoulder position is " + getShoulderPosition().in(Rotations));
        
    }

    private void manageShoulderPID()
    {
        if (m_shoulderSetpointMoveInProgress)
        {
            Angle position = getShoulderPosition();
            m_calculatedShoulderError = position.minus(m_shoulderSetpoint);
            System.out.println("Shoulder PID move in progress; at position " + position.in(Rotations) + ", setpoint is " + m_shoulderSetpoint.in(Rotations));


            m_shoulderFeedforwardVoltage = m_shoulderFeedforward.calculate((position).in(Radians) + Math.PI / 2, getShoulderVelocity().in(RadiansPerSecond));

            m_shoulderErrorDerivative = m_shoulderController.getErrorDerivative();
            m_shoulderAccumulatedError = m_shoulderController.getAccumulatedError();
            m_shoulderController.getError();
            double voltage = m_shoulderController.calculate(getShoulderPosition().in(Rotations), m_shoulderSetpoint.in(Rotations));
            System.out.println("calculated voltage before adding is " + voltage);

            //voltage = voltage + Math.abs(shoulderFeedforwardVoltage) * Math.signum(voltage);  //TODO disable feed forward for now until we get things stable


            if (Constants.ArmConstants.SHOULDER_MOTOR_IS_INVERTED) {
                voltage = -1 * voltage;
            }

            if (m_shoulderController.atSetpoint()) {
                m_shoulderSetpointMoveInProgress = false;
                voltage = 0.0;
            }

            //TODO we need to check these values
            if (position.in(Rotations) >= 0.05 || position.in(Rotations) <= -0.4) {
                voltage = 0.0;
                m_shoulderSetpointMoveInProgress = false;
                System.out.println("Detected illegal arm position.  Stopping PID shoulder operation.  Position: " + position.in(Rotations));
            }

            System.out.println("setting shoulder voltage to " + voltage);
            m_shoulderMotor.setVoltage(voltage);
        }
    }

    private void updateTelemetry()
    {
        m_rawShoulderPositionPublisher.set(getShoulderPosition().in(Rotations));
        m_rawShoulderVelocityPublisher.set(getShoulderVelocity().in(RotationsPerSecond));
        m_rawShoulderSetpointPublisher.set(m_shoulderSetpoint.in(Rotations));
        m_rawShoulderErrorPublisher.set(m_calculatedShoulderError.in(Rotations));
        m_rawShoulderDerivativePublisher.set(m_shoulderErrorDerivative);
        m_rawShoulderVoltagePublisher.set(m_shoulderMotor.getVoltage());
        m_rawShoulderFeedforwardPublisher.set(m_shoulderFeedforwardVoltage);        
        m_rawWristPositionPublisher.set(m_wristEncoder.getPosition().in(Rotations));
        m_rawWristVelocityPublisher.set(m_wristEncoder.getVelocity().in(RotationsPerSecond));
        m_rawWristSetpointPublisher.set(wristSetpoint.in(Rotations));
        m_rawWristErrorPublisher.set(wristError.in(Rotations));
        m_rawWristVoltagePublisher.set(m_wristMotor.getVoltage());
        m_shoulderErrorAccumulation.set(m_shoulderAccumulatedError);
    }

    public ArmFeedforward getDefaultShoulderFeedForward() {
        return ArmMath.createShoulderFeedforward();
    }
    public ArmFeedforward getDefaultWristFeedForward() {
        return ArmMath.createWristFeedforward();
    }

    public void periodic() {
        
        manageShoulderPID();
        
        updateTelemetry();
    }
}
