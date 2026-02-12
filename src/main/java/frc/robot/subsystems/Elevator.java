package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import frc.robot.Constants.ElevatorConstants;

/*
 * Guidelines for setting up the elevator subsystem:
 * 1. Identify the leader motor on the elevator by making it blink.
 * 2. Determine the rotation direction by manually operating the elevator.
 * 3. Set the inversion based on the required direction for upward movement.
 * 
 * 4. Proceed with additional calibration as needed.
 */

public class Elevator extends SubsystemBase {

    private final TalonFX primaryMotor = new TalonFX(ElevatorConstants.kElevatorLeaderID);
    private final TalonFX secondaryMotor = new TalonFX(ElevatorConstants.kElevatorFollowerID);
    private final CANcoder absoluteEncoder = new CANcoder(ElevatorConstants.kCANCoderID);
    private final DigitalInput bottomSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

    public Elevator() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
        outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        outputConfig.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput = outputConfig;

        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.RotorToSensorRatio = 1;
        feedbackConfig.SensorToMechanismRatio = 1;
        feedbackConfig.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback = feedbackConfig;

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit = Amps.of(40).in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits = currentConfig;

        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = RotationsPerSecond.of(12).in(RotationsPerSecond);
        motionMagicConfig.MotionMagicAcceleration = RotationsPerSecondPerSecond.of(10).in(RotationsPerSecondPerSecond);
        motionMagicConfig.MotionMagicJerk = RotationsPerSecondPerSecond.per(Second).of(0).in(RotationsPerSecondPerSecond.per(Second));
        motorConfig.MotionMagic = motionMagicConfig;

        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = 5.0;
        slotConfig.kI = 2.0;
        slotConfig.kD = 0.0;
        slotConfig.kG = 0.2;
        slotConfig.kS = 0.0;
        slotConfig.kV = 0.0;
        slotConfig.kA = 0.0;
        motorConfig.Slot0 = slotConfig;

        primaryMotor.getConfigurator().apply(motorConfig);
        secondaryMotor.getConfigurator().apply(motorConfig);

        secondaryMotor.setControl(new Follower(ElevatorConstants.kElevatorLeaderID, true));

        primaryMotor.getConfigurator().setPosition(0);
        secondaryMotor.getConfigurator().setPosition(0);
    }

    public boolean getLimitSwitch() {
        return !bottomSwitch.get();
    }

    public void moveMotor(double rotations) {
        primaryMotor.setControl(motionRequest.withPosition(rotations));
    }

    public Command moveMotorCommand(double rotations) {
        return run(() -> moveMotor(rotations));
    }

    public Command goLimpCommand() {
        return run(() -> primaryMotor.setControl(new DutyCycleOut(0)));
    }

    public Command level1() {
        return moveMotorCommand(0.2);
    }

    public Command level2() {
        return moveMotorCommand(0.6);
    }

    public Command level3() {
        return moveMotorCommand(1.2);
    }

    public Command approachZero() {
        return moveMotorCommand(0.01);
    }

    public double getLeaderEncoderPosition() {
        return primaryMotor.getPosition().getValueAsDouble();
    }

    public boolean closeToZero() {
        return getLeaderEncoderPosition() < 0.02;
    }

    public Command goToZero() {
        return Commands.sequence(
            moveMotorCommand(0.05).withTimeout(4),
            goLimpCommand()
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Primary Motor Rotations", primaryMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Secondary Motor Rotations", secondaryMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    }
}