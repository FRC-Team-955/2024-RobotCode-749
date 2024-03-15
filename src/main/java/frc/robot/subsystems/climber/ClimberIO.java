package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIO {
    public static ClimberIOInputs inputs() {
        // Kotlin compiles before Java, so the generated class doesn't always exist. To make this work we also have to make the inputs class abstract and implement LoggableInputs, otherwise kotlin will complain when we try to call Logger.processInputs()
        return new ClimberIOInputsAutoLogged();
    }

    @AutoLog // @AutoLog doesn't work with Kotlin classes, so this file has to be Java
    public static abstract class ClimberIOInputs implements LoggableInputs {
        public double positionRad = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public void updateInputs(ClimberIOInputs inputs) {
    }

    public void set(double volts) {
    }

    public void stop() {
    }

    public void resetPosition() {
    }
}
