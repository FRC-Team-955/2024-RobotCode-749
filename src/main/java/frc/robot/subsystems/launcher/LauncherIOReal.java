package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.constants.LauncherConstants;

public class LauncherIOReal extends LauncherIO {
    private final CANSparkMax top = new CANSparkMax(LauncherConstants.topMotorId, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax bottom = new CANSparkMax(LauncherConstants.bottomMotorId, CANSparkLowLevel.MotorType.kBrushed);

    @Override
    public void setTopVoltage(double volts) {
        top.set(volts);
    }

    @Override
    public void setBottomVoltage(double volts) {
        bottom.set(volts);
    }

    @Override
    public void stop() {
        top.stopMotor();
        bottom.stopMotor();
    }
}
