package frc.robot.subsystems.climber

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.util.Units
import frc.robot.Constants

class ClimberIORealLeft :
    ClimberIOReal(CANSparkMax(Constants.Climber.leftMotorId, CANSparkLowLevel.MotorType.kBrushless)) {
}

class ClimberIORealRight :
    ClimberIOReal(CANSparkMax(Constants.Climber.rightMotorId, CANSparkLowLevel.MotorType.kBrushless)) {
}

abstract class ClimberIOReal(private val motor: CANSparkMax) : ClimberIO() {
    private val encoder: RelativeEncoder = motor.encoder

    init {
        motor.restoreFactoryDefaults()
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        motor.setCANTimeout(250)
        motor.enableVoltageCompensation(12.0)
        motor.setSmartCurrentLimit(40)
        motor.burnFlash()
        encoder.setPosition(0.0)
    }

    override fun updateInputs(inputs: ClimberIOInputs) {
        inputs.appliedVolts = motor.appliedOutput * motor.busVoltage
        inputs.currentAmps = motor.outputCurrent
        inputs.positionRad = Units.rotationsToRadians(encoder.position / Constants.Climber.gearRatio)
    }

    override fun set(volts: Double) {
        motor.setVoltage(volts)
    }

    override fun stop() {
        motor.stopMotor()
    }

    override fun resetPosition() {
        encoder.setPosition(0.0)
    }
}
