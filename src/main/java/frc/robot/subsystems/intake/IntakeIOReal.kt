package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants


class IntakeIOReal : IntakeIO() {
    private val pivot = CANSparkMax(Constants.Intake.pivotMotorId, CANSparkLowLevel.MotorType.kBrushless)
    private val driver = CANSparkMax(Constants.Intake.driverMotorId, CANSparkLowLevel.MotorType.kBrushless)

    private val pivotEncoder = pivot.encoder
    private val limitSwitch = DigitalInput(Constants.Intake.limitSwitchPort)
    private var limitSwitchDebouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)

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

        inputs.hasNote = !limitSwitchDebouncer.calculate(limitSwitch.get())
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
