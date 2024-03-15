package frc.robot.subsystems.climber

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.Constants

class ClimberIORealRight : ClimberIOReal() {
    override fun getMotor(): CANSparkMax {
        return CANSparkMax(Constants.Climber.rightMotorId, CANSparkLowLevel.MotorType.kBrushless)
    }
}
