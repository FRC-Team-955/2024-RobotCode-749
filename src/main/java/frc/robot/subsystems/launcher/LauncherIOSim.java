package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class LauncherIOSim extends LauncherIO {
    private final DCMotorSim top = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);
    private final DCMotorSim bottom = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);

    private double topAppliedVolts = 0.0;
    private double bottomAppliedVolts = 0.0;

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        top.update(0.02);
        bottom.update(0.02);

        inputs.topAppliedVolts = topAppliedVolts;
        inputs.topCurrentAmps = new double[]{top.getCurrentDrawAmps()};

        inputs.bottomAppliedVolts = bottomAppliedVolts;
        inputs.bottomCurrentAmps = new double[]{bottom.getCurrentDrawAmps()};
    }

    @Override
    public void setTopVoltage(double volts) {
        topAppliedVolts = volts;
        top.setInputVoltage(volts);
    }

    @Override
    public void setBottomVoltage(double volts) {
        bottomAppliedVolts = volts;
        bottom.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        topAppliedVolts = 0;
        bottomAppliedVolts = 0;
        top.setInputVoltage(0);
        bottom.setInputVoltage(0);
    }
}
