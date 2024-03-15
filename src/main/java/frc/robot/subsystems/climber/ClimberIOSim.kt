package frc.robot.subsystems.climber

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.robot.Constants

class ClimberIOSim : ClimberIO() {
    private val motor = DCMotorSim(DCMotor.getNEO(1), Constants.Climber.gearRatio, 0.0001)
    private var appliedVolts = 0.0

    override fun updateInputs(inputs: ClimberIOInputs) {
        motor.update(0.02)

        inputs.positionRad = motor.angularPositionRad
        inputs.appliedVolts = appliedVolts
        inputs.currentAmps = motor.currentDrawAmps
    }

    override fun set(volts: Double) {
        appliedVolts = volts
        motor.setInputVoltage(volts)
    }

    override fun stop() {
        appliedVolts = 0.0
        motor.setInputVoltage(0.0)
    }

    override fun resetPosition() {
        motor.setState(0.0, 0.0)
    }
}
