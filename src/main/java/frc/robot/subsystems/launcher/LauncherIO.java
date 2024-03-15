package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LauncherIO {
    public static LauncherIOInputs inputs() {
        // Kotlin compiles before Java, so the generated class doesn't always exist if we use it in Kotlin code. We also have to make the inputs class abstract and implement LoggableInputs, otherwise kotlin will complain when we try to call Logger.processInputs()
        return new LauncherIOInputsAutoLogged();
    }

    @AutoLog // @AutoLog doesn't work with Kotlin classes, so this file has to be Java
    public static abstract class LauncherIOInputs implements LoggableInputs {
        public double topAppliedVolts = 0.0;
        public double topCurrentAmps = 0.0;

        public double bottomAppliedVolts = 0.0;
        public double bottomCurrentAmps = 0.0;
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
