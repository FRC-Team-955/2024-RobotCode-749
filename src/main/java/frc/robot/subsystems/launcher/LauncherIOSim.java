package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class LauncherIOSim extends LauncherIO {
    private final DCMotorSim top = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);
    private final DCMotorSim bottom = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);

    @Override
    public void setTopVoltage(double volts) {
        top.setInputVoltage(volts);
    }

    @Override
    public void setBottomVoltage(double volts) {
        bottom.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        top.setInputVoltage(0);
        bottom.setInputVoltage(0);
    }
}
