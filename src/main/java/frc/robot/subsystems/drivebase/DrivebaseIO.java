package frc.robot.subsystems.drivebase;

import org.littletonrobotics.junction.AutoLog;

public class DrivebaseIO {
    @AutoLog
    public static class DrivebaseIOInputs {
        public double leftEncoderDistance = 0.0;
        public double leftEncoderRate = 0.0;
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftLeaderCurrentAmps = 0.0;
        public double leftFollowerCurrentAmps = 0.0;

        public double rightEncoderDistance = 0.0;
        public double rightEncoderRate = 0.0;
        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightLeaderCurrentAmps = 0.0;
        public double rightFollowerCurrentAmps = 0.0;
    }

    public void updateInputs(DrivebaseIOInputs inputs) {
    }

    public void setVoltage(double leftVolts, double rightVolts) {
    }
}
