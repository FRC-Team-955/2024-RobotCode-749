package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.util.Units
import frc.robot.Constants

class IntakeIOReal : IntakeIO() {
    private val pivot = CANSparkMax(Constants.Intake.pivotMotorId, CANSparkLowLevel.MotorType.kBrushless)
    private val driver = CANSparkMax(Constants.Intake.driverMotorId, CANSparkLowLevel.MotorType.kBrushless)

    private val pivotEncoder: RelativeEncoder = pivot.encoder

    init {
        pivot.restoreFactoryDefaults()
        driver.restoreFactoryDefaults()

        pivot.setIdleMode(CANSparkBase.IdleMode.kBrake)
        driver.setIdleMode(CANSparkBase.IdleMode.kCoast)

        pivot.setCANTimeout(250)
        driver.setCANTimeout(250)

        pivot.enableVoltageCompensation(12.0)
        driver.enableVoltageCompensation(12.0)
        pivot.setSmartCurrentLimit(40)
        driver.setSmartCurrentLimit(40)

        pivot.burnFlash()
        driver.burnFlash()

        pivotEncoder.setPosition(0.0)
    }

    override fun updateInputs(inputs: IntakeIOInputs) {
        inputs.pivotPositionRad = Units.rotationsToRadians(pivotEncoder.position / Constants.Intake.pivotGearRatio)
        inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.velocity)
        inputs.pivotAppliedVolts = pivot.appliedOutput * pivot.busVoltage
        inputs.pivotCurrentAmps = pivot.outputCurrent

        inputs.driverAppliedVolts = driver.appliedOutput * driver.busVoltage
        inputs.driverCurrentAmps = driver.outputCurrent
    }

    override fun setPivotVoltage(volts: Double) {
        pivot.setVoltage(volts)
    }

    override fun resetPivotPosition() {
        pivotEncoder.setPosition(0.0)
    }

    override fun setDriverVoltage(volts: Double) {
        driver.setVoltage(volts)
    }

    override fun stopDriver() {
        driver.stopMotor()
    }
}
