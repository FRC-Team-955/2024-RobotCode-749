package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
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

    private final SparkPIDController leftPID = leftLeader.getPIDController();
    private final SparkPIDController rightPID = rightLeader.getPIDController();

    private final Pigeon2 pigeon = new Pigeon2(DrivebaseConstants.pigeonId);
    private final StatusSignal<Double> yaw = pigeon.getYaw();

    public DrivebaseIOReal() {
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

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
        leftLeader.setSmartCurrentLimit(60);
        rightLeader.setSmartCurrentLimit(60);

        leftPID.setP(DrivebaseConstants.motorP);
        leftPID.setD(DrivebaseConstants.motorD);
        rightPID.setP(DrivebaseConstants.motorP);
        rightPID.setD(DrivebaseConstants.motorD);

        leftLeader.burnFlash();
        rightLeader.burnFlash();
        leftFollower.burnFlash();
        rightFollower.burnFlash();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
        inputs.leftEncoderDistance = leftEncoder.getDistance();
        inputs.leftEncoderRate = leftEncoder.getRate();
        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getDistance());
        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getRate());
        inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
        inputs.leftLeaderCurrentAmps = leftLeader.getOutputCurrent();
        inputs.leftFollowerCurrentAmps = leftFollower.getOutputCurrent();

        inputs.rightEncoderDistance = rightEncoder.getDistance();
        inputs.rightEncoderRate = rightEncoder.getRate();
        inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getDistance());
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getRate());
        inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * rightLeader.getBusVoltage();
        inputs.rightLeaderCurrentAmps = rightLeader.getOutputCurrent();
        inputs.rightFollowerCurrentAmps = rightFollower.getOutputCurrent();

        NetworkTable leftLimelight = NetworkTableInstance.getDefault().getTable("limelight-left");
        inputs.leftLimelightTv = leftLimelight.getEntry("tv").getInteger(-1);
        double[] leftBotpose = leftLimelight.getEntry("botpose").getDoubleArray((double[]) null);
        if (leftBotpose != null) {
            inputs.leftLimelightBotpose = new Pose2d(leftBotpose[0], leftBotpose[1], Rotation2d.fromDegrees(leftBotpose[5]));
            inputs.leftLimelightBotposeTimestamp = Timer.getFPGATimestamp() - (leftBotpose[6] / 1000.0);
        }

        NetworkTable rightLimelight = NetworkTableInstance.getDefault().getTable("limelight-right");
        inputs.rightLimelightTv = rightLimelight.getEntry("tv").getInteger(-1);
        double[] rightLimelightBotpose = rightLimelight.getEntry("botpose").getDoubleArray((double[]) null);
        if (rightLimelightBotpose != null) {
            inputs.rightLimelightBotpose = new Pose2d(rightLimelightBotpose[0], rightLimelightBotpose[1], Rotation2d.fromDegrees(rightLimelightBotpose[5]));
            inputs.rightLimelightBotposeTimestamp = Timer.getFPGATimestamp() - (rightLimelightBotpose[6] / 1000.0);
        }

        inputs.gyroYaw = Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble());
    }

    private void setLimelightInputs(String name, long tv, Pose2d botpose, double botposeTimestamp) {
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
        leftPID.setReference(
                Units.radiansPerSecondToRotationsPerMinute(leftRadPerSec * DrivebaseConstants.gearRatio.value),
                CANSparkBase.ControlType.kVelocity,
                0,
                leftFFVolts
        );
        rightPID.setReference(
                Units.radiansPerSecondToRotationsPerMinute(rightRadPerSec * DrivebaseConstants.gearRatio.value),
                CANSparkBase.ControlType.kVelocity,
                0,
                rightFFVolts
        );
    }
}
