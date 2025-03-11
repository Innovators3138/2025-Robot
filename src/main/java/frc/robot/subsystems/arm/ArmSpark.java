package frc.robot.subsystems.arm;

import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.utils.PIDFConfig;

public class ArmSpark implements ArmIO{

    private static final int maximumRetries = 5;
    private SparkBaseConfig cfg;
    private final SparkBase motor;
    public SparkClosedLoopController pid;

    public ArmSpark(SparkBase motor, SparkBaseConfig cfg, DCMotor motorType)
    {
        this.motor = motor;
        this.cfg = cfg;

        pid = motor.getClosedLoopController();

        configureSpark(motor::clearFaults);
    }

    public void configureSpark(Supplier<REVLibError> config)
    {
        for (int i =0; i < maximumRetries; i++)
        {
            if (config.get() == REVLibError.kOk)
            {
                return;
            }
            Timer.delay(Milliseconds.of(5).in(Seconds));
        }
        DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
    }

    public void updateConfig(SparkBaseConfig motorCfg, SparkBaseConfig cfgGiven)
    {
        if (!DriverStation.isDisabled())
        {
            throw new RuntimeException("Configuration changes cannot be applied while the robot is enabled.");
        }
        motorCfg.apply(cfgGiven);
        configureSpark(() -> motor.configure(motorCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    }

    public void setVoltageCompensation(double nominalVoltage)
    {
        cfg.voltageCompensation(nominalVoltage);
    }

    public void setCurrentLimit(int currentLimit)
    {
        cfg.smartCurrentLimit(currentLimit);
    }

    public void setLoopRampRate(double rampRate)
    {
        cfg.closedLoopRampRate(rampRate)
           .openLoopRampRate(rampRate);
    }

    public Object getMotor()
    {
        return motor;
    }

    public void configurePIDF(PIDFConfig config)
    {
        cfg.closedLoop.pidf(config.p, config.i, config.d, config.f)
                      .iZone(config.iz)
                      .outputRange(config.output.min, config.output.max);
    }

    public void configurePIDWrapping(double minInput, double maxInput)
    {
        cfg.closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(minInput, maxInput);
  
    }

    public void setMotorBrake(boolean isBrakeMode)
    {
        cfg.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setInverted(boolean inverted)
    {
        cfg.inverted(inverted);
    }

    public void burnFlash()
    {
        if (!DriverStation.isDisabled())
        {
        throw new RuntimeException("Config updates cannot be applied while the robot is Enabled!");
        }
        configureSpark(() -> {
            return motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        });
    }

    public void set(double percentOutput)
    {
        motor.set(percentOutput);
    }

    public void setReference(double position, double feedforward)
            {
                configureSpark(() -> pid.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward));
    }

    public void setReference(double setpoint, double feedforward, Angle position)
    {
        setReference(position, feedforward);
            }
        
            void setReference(Angle position, double feedforward) {
                throw new UnsupportedOperationException("Unimplemented method 'setReference'");
            }
        
            public double getVoltage()
    {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public void setVoltage(double voltage)
    {
        motor.setVoltage(voltage);
    }

    public double getAppliedOutput()
    {
        return motor.getAppliedOutput();
    }

    public double getOutputCurrent()
    {
        return motor.getOutputCurrent();
    }

    public double getMotorTemperature()
    {
        return motor.getMotorTemperature();
    }

    public void updateInputs(ArmIOInputs inputs, ArmSpark shoulderMotor, ArmSpark wristMotor, ArmEncoder shoulderEncoder, ArmEncoder wristEncoder) {
        inputs.shoulderPositionRad = shoulderEncoder.getPosition().in(Radians);
        inputs.wristPositionRad = wristEncoder.getPosition().in(Radians);
        inputs.shoulderVelocityRadPerSec = shoulderEncoder.getVelocity().in(RadiansPerSecond);
        inputs.wristVelocityRadPerSec = wristEncoder.getVelocity().in(RadiansPerSecond);  
        inputs.shoulderAppliedVolts = shoulderMotor.getVoltage();
        inputs.wristAppliedVolts = wristMotor.getVoltage();
        inputs.shoulderSupplyCurrentAmps = shoulderMotor.getOutputCurrent();
        inputs.wristSupplyCurrentAmps = wristMotor.getOutputCurrent();
        inputs.shoulderTempCelsius = shoulderMotor.getMotorTemperature();
        inputs.wristTempCelsius = wristMotor.getMotorTemperature();
    }

}

