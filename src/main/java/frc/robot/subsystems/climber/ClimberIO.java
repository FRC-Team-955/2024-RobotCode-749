package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public class ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
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
