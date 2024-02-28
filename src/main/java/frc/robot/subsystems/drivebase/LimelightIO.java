package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class LimelightIO {
    @AutoLog
    public static class LimelightIOInputs {
        public long leftTv = -1;
        public Pose2d leftBotpose = noBotpose();
        public double leftBotposeTimestamp = 0.0;
        public double leftTagCount = 0.0;
        public double leftAvgArea = 0.0;

        public long rightTv = -1;
        public Pose2d rightBotpose = noBotpose();
        public double rightBotposeTimestamp = 0.0;
        public double rightTagCount = 0.0;
        public double rightAvgArea = 0.0;
    }

    protected static Pose2d noBotpose() {
        return new Pose2d(-5, -5, new Rotation2d());
    }

    public void updateInputs(LimelightIOInputs inputs) {
    }
}
