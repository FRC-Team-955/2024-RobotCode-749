package frc.robot.subsystems.launcher;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.constants.LauncherConstants;

public class LauncherIOReal extends LauncherIO {
    private final WPI_TalonSRX top = new WPI_TalonSRX(LauncherConstants.topMotorId);
    private final WPI_TalonSRX bottom = new WPI_TalonSRX(LauncherConstants.bottomMotorId);

    public LauncherIOReal() {
        var config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 80;
        config.peakCurrentDuration = 250;
        config.continuousCurrentLimit = 60;
        config.voltageCompSaturation = 12.0;
        top.configAllSettings(config);
        bottom.configAllSettings(config);
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        inputs.topAppliedVolts = top.getMotorOutputVoltage();
        inputs.topCurrentAmps = top.getStatorCurrent();

        inputs.bottomAppliedVolts = bottom.getMotorOutputVoltage();
        inputs.bottomCurrentAmps = bottom.getStatorCurrent();
    }

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
