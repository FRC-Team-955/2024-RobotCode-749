package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class DrivebaseIO {
    @AutoLog
    public static class DrivebaseIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;

        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;

        public Rotation2d gyroYaw = new Rotation2d();
    }

    public void updateInputs(DrivebaseIOInputs inputs) {
    }

    public void setVoltage(double leftVolts, double rightVolts) {
    }

    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    }
}
