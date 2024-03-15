package frc.robot.subsystems.intake

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.Constants

class IntakeIOSim : IntakeIO() {
    private val pivot = SingleJointedArmSim(
        DCMotor.getNEO(1),
        Constants.Intake.pivotGearRatio,
        0.0001,
        0.3,
        -Constants.Intake.pivotRadDown,
        0.0,
        true,
        0.0
    )
    private val driver = DCMotorSim(DCMotor.getNEO(1), 1.0, 0.0001)

    private var pivotAppliedVolts = 0.0
    private var driverAppliedVolts = 0.0

    override fun updateInputs(inputs: IntakeIOInputs) {
        pivot.update(0.02)
        driver.update(0.02)

        inputs.pivotPositionRad = pivot.angleRads
        inputs.pivotVelocityRadPerSec = pivot.velocityRadPerSec
        inputs.pivotAppliedVolts = pivotAppliedVolts
        inputs.pivotCurrentAmps = pivot.currentDrawAmps

        inputs.driverAppliedVolts = driverAppliedVolts
        inputs.driverCurrentAmps = driver.currentDrawAmps
    }

    override fun setPivotVoltage(volts: Double) {
        pivotAppliedVolts = volts
        pivot.setInputVoltage(volts)
    }

    override fun resetPivotPosition() {
        pivot.setState(0.0, 0.0)
    }

    override fun setDriverVoltage(volts: Double) {
        driverAppliedVolts = volts
        driver.setInputVoltage(volts)
    }

    override fun stopDriver() {
        driverAppliedVolts = 0.0
        driver.setInputVoltage(0.0)
    }
}
