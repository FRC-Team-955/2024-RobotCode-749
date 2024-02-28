package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.constants.ClimberConstants;

public class ClimberIORealLeft extends ClimberIOReal {
    @Override
    protected CANSparkMax getMotor() {
        return new CANSparkMax(ClimberConstants.leftMotorId, CANSparkLowLevel.MotorType.kBrushless);
    }
}
