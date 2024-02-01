package frc.robot.subsystems.launcher;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.constants.LauncherConstants;

public class LauncherIOReal extends LauncherIO {
    private final WPI_TalonSRX top = new WPI_TalonSRX(LauncherConstants.topMotorId);
    private final WPI_TalonSRX bottom = new WPI_TalonSRX(LauncherConstants.bottomMotorId);

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
