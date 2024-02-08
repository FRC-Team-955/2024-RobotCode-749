package frc.robot.subsystems.drivebase;

import com.revrobotics.*;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DrivebaseConstants;

public class DrivebaseIOReal extends DrivebaseIO {
    private final CANSparkMax leftLeader = new CANSparkMax(DrivebaseConstants.leftLeaderMotorId, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax leftFollower = new CANSparkMax(DrivebaseConstants.leftFollowerMotorId, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax rightLeader = new CANSparkMax(DrivebaseConstants.rightLeaderMotorId, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax rightFollower = new CANSparkMax(DrivebaseConstants.rightFollowerMotorId, CANSparkLowLevel.MotorType.kBrushed);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

    private final SparkPIDController leftPID = leftLeader.getPIDController();
    private final SparkPIDController rightPID = rightLeader.getPIDController();

    public DrivebaseIOReal() {
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftLeader.setCANTimeout(250);
        rightLeader.setCANTimeout(250);
        leftFollower.setCANTimeout(250);
        rightFollower.setCANTimeout(250);

        leftLeader.setInverted(false);
        rightLeader.setInverted(true);
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
    }

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / DrivebaseConstants.gearRatio.value);
        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / DrivebaseConstants.gearRatio.value);

        inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / DrivebaseConstants.gearRatio.value);
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / DrivebaseConstants.gearRatio.value);

//        inputs.gyroYaw = Gyro.getRotation2d();
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
