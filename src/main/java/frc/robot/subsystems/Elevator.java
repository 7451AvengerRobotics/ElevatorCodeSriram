package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

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
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(12))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0))
        ).withSlot0(
            new Slot0Configs()
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKG(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
        );

        elevatorLeader.getConfigurator().apply(cfg);
        elevatorFollower.setControl(new Follower(ElevatorConstants.kElevatorLeaderID, true));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Rotations", elevatorLeader.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    }


    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public void moveMotor(double rotations) {
        elevatorLeader.setControl(elevatorRequest.withPosition(rotations));
    }
}
