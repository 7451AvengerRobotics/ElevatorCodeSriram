package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

import javax.sound.midi.VoiceStatus;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

/*
 * What to do once you come to the elevator subsystem:
 * 1. Find which motor is the leader on the elevator by blinking it
 * 2. Find what direction it turns by manually moving the elevator
 * 3. Add the withInverted based on the direction it needs to turn to go up
 * 
 * 4. 
 */


public class Elevator extends SubsystemBase {
    private final TalonFX elevatorLeader = new TalonFX(ElevatorConstants.kElevatorLeaderID);
    private final TalonFX elevatorFollower = new TalonFX(ElevatorConstants.kElevatorFollowerID);
    private final CANcoder elevatorEncoder = new CANcoder(ElevatorConstants.kCANCoderID);
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
    private final MotionMagicVoltage elevatorRequest = new MotionMagicVoltage(0);

    public Elevator() {
        TalonFXConfiguration cfg = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        ).withFeedback(
            new FeedbackConfigs()
                .withRotorToSensorRatio(1)
                .withSensorToMechanismRatio(1)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(12))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0))
        ).withSlot0(
            new Slot0Configs()
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKG(0.2)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
        );

        elevatorLeader.getConfigurator().apply(cfg);
        elevatorFollower.getConfigurator().apply(cfg);

        elevatorFollower.setControl(new Follower(ElevatorConstants.kElevatorLeaderID, true));

        elevatorLeader.getConfigurator().setPosition(0);
        elevatorFollower.getConfigurator().setPosition(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Leader Rotations", elevatorLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Follower Rotations", elevatorFollower.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    }


    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public void moveMotor(double rotations) {
        elevatorLeader.setControl(elevatorRequest.withPosition(rotations));
    }

    public Command moveMotorCommand(double rotations) {
        return run(() ->
            moveMotor(rotations)
        );
    }

    public Command goLimpCommand() {
        return run(() -> 
            elevatorLeader.setControl(new DutyCycleOut(0))
        );
    }

    public Command level1() {
        return moveMotorCommand(2);
    }

    public Command level2() {
        return moveMotorCommand(8);
    }

    public Command level3() {
        return moveMotorCommand(12);
    }

    public Command approachZero() {
        return moveMotorCommand(0.1);
    }

    public double getLeaderEncoderPosition() {
        return elevatorLeader.getPosition().getValueAsDouble();
    }

    public boolean closeToZero() {
        return getLeaderEncoderPosition() < 0.2;
    }
}
