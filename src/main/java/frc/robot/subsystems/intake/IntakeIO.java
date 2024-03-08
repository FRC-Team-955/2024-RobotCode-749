package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double pivotPositionRad = 0.0;
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double driverAppliedVolts = 0.0;
        public double driverCurrentAmps = 0.0;
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
