package frc.robot.subsystems.drivebase

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants

class DrivebaseIOReal : DrivebaseIO() {
    private val leftLeader = CANSparkMax(Constants.Drivebase.leftLeaderMotorId, CANSparkLowLevel.MotorType.kBrushed)
    private val leftFollower =
        CANSparkMax(Constants.Drivebase.leftFollowerMotorId, CANSparkLowLevel.MotorType.kBrushed)
    private val rightLeader =
        CANSparkMax(Constants.Drivebase.rightLeaderMotorId, CANSparkLowLevel.MotorType.kBrushed)
    private val rightFollower =
        CANSparkMax(Constants.Drivebase.rightFollowerMotorId, CANSparkLowLevel.MotorType.kBrushed)

    private val leftEncoder = run {
        val encoder = Encoder(Constants.Drivebase.leftEncoderChannelA, Constants.Drivebase.leftEncoderChannelB)
        encoder.distancePerPulse = Constants.Drivebase.encoderDistancePerPulse
        encoder
    }
    private val rightEncoder = run {
        val encoder =
            Encoder(Constants.Drivebase.rightEncoderChannelA, Constants.Drivebase.rightEncoderChannelB)
        encoder.distancePerPulse = Constants.Drivebase.encoderDistancePerPulse
        encoder.setReverseDirection(true)
        encoder
    }

    init {
        leftLeader.restoreFactoryDefaults()
        rightLeader.restoreFactoryDefaults()
        leftFollower.restoreFactoryDefaults()
        rightFollower.restoreFactoryDefaults()

        leftLeader.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rightLeader.setIdleMode(CANSparkBase.IdleMode.kBrake)
        leftFollower.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rightFollower.setIdleMode(CANSparkBase.IdleMode.kBrake)

        leftLeader.setCANTimeout(250)
        rightLeader.setCANTimeout(250)
        leftFollower.setCANTimeout(250)
        rightFollower.setCANTimeout(250)

        leftLeader.inverted = true
        rightLeader.inverted = false
        leftFollower.follow(leftLeader, false)
        rightFollower.follow(rightLeader, false)

        leftLeader.enableVoltageCompensation(12.0)
        rightLeader.enableVoltageCompensation(12.0)
        leftLeader.setSmartCurrentLimit(40)
        rightLeader.setSmartCurrentLimit(40)

        leftLeader.burnFlash()
        rightLeader.burnFlash()
        leftFollower.burnFlash()
        rightFollower.burnFlash()

        SmartDashboard.putData("Drivebase Left", leftEncoder)
        SmartDashboard.putData("Drivebase Right", rightEncoder)
    }

    override fun updateInputs(inputs: DrivebaseIOInputs) {
        inputs.leftEncoderDistance = leftEncoder.distance
        inputs.leftEncoderRate = leftEncoder.rate
        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.distance)
        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.rate * 60)
        inputs.leftAppliedVolts = leftLeader.appliedOutput * leftLeader.busVoltage
        inputs.leftLeaderCurrentAmps = leftLeader.outputCurrent
        inputs.leftFollowerCurrentAmps = leftFollower.outputCurrent

        inputs.rightEncoderDistance = rightEncoder.distance
        inputs.rightEncoderRate = rightEncoder.rate
        inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.distance)
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.rate * 60)
        inputs.rightAppliedVolts = rightLeader.appliedOutput * rightLeader.busVoltage
        inputs.rightLeaderCurrentAmps = rightLeader.outputCurrent
        inputs.rightFollowerCurrentAmps = rightFollower.outputCurrent
    }

    override fun setVoltage(leftVolts: Double, rightVolts: Double) {
        leftLeader.setVoltage(leftVolts)
        rightLeader.setVoltage(rightVolts)
    }
}
