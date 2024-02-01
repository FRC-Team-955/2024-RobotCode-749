package frc.robot.subsystems.drivebase;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.constants.DrivebaseConstants;

public class DrivebaseIOReal extends DrivebaseIO {
    private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(DrivebaseConstants.leftLeaderMotorId);
    private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(DrivebaseConstants.leftFollowerMotorId);
    private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(DrivebaseConstants.rightLeaderMotorId);
    private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(DrivebaseConstants.rightFollowerMotorId);

//    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
//    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
//
//    private final SparkPIDController leftPID = leftLeader.getPIDController();
//    private final SparkPIDController rightPID = rightLeader.getPIDController();

    public DrivebaseIOReal() {
        leftLeader.setInverted(false);
        rightLeader.setInverted(true);
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        var config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 60;
        config.peakCurrentDuration = 250;
        config.continuousCurrentLimit = 40;
        config.voltageCompSaturation = 12.0;
        leftLeader.configAllSettings(config);
        rightLeader.configAllSettings(config);
        leftFollower.configAllSettings(config);
        rightFollower.configAllSettings(config);

//        leftPID.setP(DrivebaseConstants.motorP);
//        leftPID.setD(DrivebaseConstants.motorD);
//        rightPID.setP(DrivebaseConstants.motorP);
//        rightPID.setD(DrivebaseConstants.motorD);
    }

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
//        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / DrivebaseConstants.gearRatio.value);
//        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / DrivebaseConstants.gearRatio.value);
//
//        inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / DrivebaseConstants.gearRatio.value);
//        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / DrivebaseConstants.gearRatio.value);

//        inputs.gyroYaw = Gyro.getRotation2d();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
//        leftPID.setReference(
//                Units.radiansPerSecondToRotationsPerMinute(leftRadPerSec * DrivebaseConstants.gearRatio.value),
//                CANSparkBase.ControlType.kVelocity,
//                0,
//                leftFFVolts
//        );
//        rightPID.setReference(
//                Units.radiansPerSecondToRotationsPerMinute(rightRadPerSec * DrivebaseConstants.gearRatio.value),
//                CANSparkBase.ControlType.kVelocity,
//                0,
//                rightFFVolts
//        );
    }
}
