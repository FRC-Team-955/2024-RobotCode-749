package frc.robot.subsystems.launcher

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import frc.robot.Constants
import frc.robot.Robot

class LauncherIOReal : LauncherIO() {
    private val top = WPI_TalonSRX(Constants.Launcher.topMotorId)
    private val bottom = WPI_TalonSRX(Constants.Launcher.bottomMotorId)

    init {
        val config = TalonSRXConfiguration()
        config.peakCurrentLimit = 80
        config.peakCurrentDuration = 250
        config.continuousCurrentLimit = 40
        config.voltageCompSaturation = 12.0
        top.configAllSettings(config)
        bottom.configAllSettings(config)
        top.setNeutralMode(NeutralMode.Brake)
        bottom.setNeutralMode(NeutralMode.Brake)
    }

    override fun updateInputs(inputs: LauncherIOInputs) {
        inputs.topAppliedVolts = top.motorOutputVoltage
        inputs.topCurrentAmps = top.statorCurrent

        inputs.bottomAppliedVolts = bottom.motorOutputVoltage
        inputs.bottomCurrentAmps = bottom.statorCurrent
    }

    override fun setTopVoltage(volts: Double) {
        if (!Robot.lowPowerMode.get()) {
            top.setVoltage(volts)
        } else {
            top.stopMotor()
        }
    }

    override fun setBottomVoltage(volts: Double) {
        if (!Robot.lowPowerMode.get()) {
            bottom.setVoltage(volts)
        } else {
            bottom.stopMotor()
        }
    }

    override fun stop() {
        top.stopMotor()
        bottom.stopMotor()
    }
}
