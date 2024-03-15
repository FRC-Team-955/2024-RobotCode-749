package frc.robot.subsystems.climber

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.Constants

class ClimberIORealLeft : ClimberIOReal() {
    override fun getMotor(): CANSparkMax {
        return CANSparkMax(Constants.Climber.leftMotorId, CANSparkLowLevel.MotorType.kBrushless)
    }
}
