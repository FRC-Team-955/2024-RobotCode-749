package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

        public long leftLimelightTv = -1;
        public Pose2d leftLimelightBotpose = new Pose2d();
        public double leftLimelightBotposeTimestamp = 0.0;

        public long rightLimelightTv = -1;
        public Pose2d rightLimelightBotpose = new Pose2d();
        public double rightLimelightBotposeTimestamp = 0.0;

        public Rotation2d gyroYaw = new Rotation2d();
    }

    public void updateInputs(DrivebaseIOInputs inputs) {
    }

    public void setVoltage(double leftVolts, double rightVolts) {
    }
}
