package frc.robot.subsystems.drivebase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util;
import frc.robot.constants.DrivebaseConstants;

public class DrivebaseIOReal extends DrivebaseIO {
    private final CANSparkMax leftLeader = new CANSparkMax(DrivebaseConstants.leftLeaderMotorId, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax leftFollower = new CANSparkMax(DrivebaseConstants.leftFollowerMotorId, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax rightLeader = new CANSparkMax(DrivebaseConstants.rightLeaderMotorId, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax rightFollower = new CANSparkMax(DrivebaseConstants.rightFollowerMotorId, CANSparkLowLevel.MotorType.kBrushed);

    private final Encoder leftEncoder = Util.make(() -> {
        var encoder = new Encoder(DrivebaseConstants.leftEncoderChannelA, DrivebaseConstants.leftEncoderChannelB);
        encoder.setDistancePerPulse(DrivebaseConstants.encoderDistancePerPulse);
        return encoder;
    });
    private final Encoder rightEncoder = Util.make(() -> {
        var encoder = new Encoder(DrivebaseConstants.rightEncoderChannelA, DrivebaseConstants.rightEncoderChannelB);
        encoder.setDistancePerPulse(DrivebaseConstants.encoderDistancePerPulse);
        encoder.setReverseDirection(true);
        return encoder;
    });

    public DrivebaseIOReal() {
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftLeader.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightLeader.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leftFollower.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightFollower.setIdleMode(CANSparkBase.IdleMode.kBrake);

        leftLeader.setCANTimeout(250);
        rightLeader.setCANTimeout(250);
        leftFollower.setCANTimeout(250);
        rightFollower.setCANTimeout(250);

        leftLeader.setInverted(true);
        rightLeader.setInverted(false);
        leftFollower.follow(leftLeader, false);
        rightFollower.follow(rightLeader, false);

        leftLeader.enableVoltageCompensation(12.0);
        rightLeader.enableVoltageCompensation(12.0);
        leftLeader.setSmartCurrentLimit(40);
        rightLeader.setSmartCurrentLimit(40);

        leftLeader.burnFlash();
        rightLeader.burnFlash();
        leftFollower.burnFlash();
        rightFollower.burnFlash();

        SmartDashboard.putData("Drivebase Left", leftEncoder);
        SmartDashboard.putData("Drivebase Right", rightEncoder);
    }

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
        inputs.leftEncoderDistance = leftEncoder.getDistance();
        inputs.leftEncoderRate = leftEncoder.getRate();
        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getDistance());
        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getRate() * 60);
        inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
        inputs.leftLeaderCurrentAmps = leftLeader.getOutputCurrent();
        inputs.leftFollowerCurrentAmps = leftFollower.getOutputCurrent();

        inputs.rightEncoderDistance = rightEncoder.getDistance();
        inputs.rightEncoderRate = rightEncoder.getRate();
        inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getDistance());
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getRate() * 60);
        inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * rightLeader.getBusVoltage();
        inputs.rightLeaderCurrentAmps = rightLeader.getOutputCurrent();
        inputs.rightFollowerCurrentAmps = rightFollower.getOutputCurrent();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }
}
