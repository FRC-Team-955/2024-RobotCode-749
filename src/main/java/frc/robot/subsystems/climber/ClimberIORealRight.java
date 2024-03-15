package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class ClimberIORealRight extends ClimberIOReal {
    @Override
    protected CANSparkMax getMotor() {
        return new CANSparkMax(Constants.Climber.rightMotorId, CANSparkLowLevel.MotorType.kBrushless);
    }
}
