package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIO {
    public static IntakeIOInputs inputs() {
        // Kotlin compiles before Java, so the generated class doesn't always exist if we use it in Kotlin code. We also have to make the inputs class abstract and implement LoggableInputs, otherwise kotlin will complain when we try to call Logger.processInputs()
        return new IntakeIOInputsAutoLogged();
    }

    @AutoLog // @AutoLog doesn't work with Kotlin classes, so this file has to be Java
    public static abstract class IntakeIOInputs implements LoggableInputs {
        public double pivotPositionRad = 0.0;
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double driverAppliedVolts = 0.0;
        public double driverCurrentAmps = 0.0;

        public boolean hasNote = false;
    }

    public void updateInputs(IntakeIOInputs inputs) {
    }

    public void setPivotVoltage(double volts) {
    }

    public void resetPivotPosition() {
    }

    public void setDriverVoltage(double volts) {
    }

    public void stopDriver() {
    }
}
