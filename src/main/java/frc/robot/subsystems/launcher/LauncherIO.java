package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public class LauncherIO {
    @AutoLog
    public static class LauncherIOInputs {
        public double topAppliedVolts = 0.0;
        public double[] topCurrentAmps = new double[]{};

        public double bottomAppliedVolts = 0.0;
        public double[] bottomCurrentAmps = new double[]{};
    }

    public void updateInputs(LauncherIOInputs inputs) {
    }

    public void setTopVoltage(double volts) {
    }

    public void setBottomVoltage(double volts) {
    }

    public void stop() {
    }
}
