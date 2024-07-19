package frc.robot.subsystems.launcher

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim

class LauncherIOSim : LauncherIO() {
    private val top = DCMotorSim(DCMotor.getCIM(1), 1.0, 0.0001)
    private val bottom = DCMotorSim(DCMotor.getCIM(1), 1.0, 0.0001)

    private var topAppliedVolts = 0.0
    private var bottomAppliedVolts = 0.0

    override fun updateInputs(inputs: LauncherIOInputs) {
        top.update(0.02)
        bottom.update(0.02)

        inputs.topAppliedVolts = topAppliedVolts
        inputs.topCurrentAmps = top.currentDrawAmps

        inputs.bottomAppliedVolts = bottomAppliedVolts
        inputs.bottomCurrentAmps = bottom.currentDrawAmps
    }

    override fun setTopVoltage(volts: Double) {
//        if (!Robot.lowPowerMode.get()) {
        topAppliedVolts = volts
        top.setInputVoltage(volts)
//        } else {
//            topAppliedVolts = 0.0
//            top.setInputVoltage(0.0)
//        }
    }

    override fun setBottomVoltage(volts: Double) {
//        if (!Robot.lowPowerMode.get()) {
        bottomAppliedVolts = volts
        bottom.setInputVoltage(volts)
//        } else {
//            bottomAppliedVolts = 0.0
//            bottom.setInputVoltage(0.0)
//        }
    }

    override fun stop() {
        topAppliedVolts = 0.0
        bottomAppliedVolts = 0.0
        top.setInputVoltage(0.0)
        bottom.setInputVoltage(0.0)
    }
}
