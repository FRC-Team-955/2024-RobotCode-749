package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public class LimelightIO {
    @AutoLog
    public static class LimelightIOInputs {
        public long leftTv = -1;
        public Pose2d leftBotpose = new Pose2d();
        public double leftBotposeTimestamp = 0.0;

        public long rightTv = -1;
        public Pose2d rightBotpose = new Pose2d();
        public double rightBotposeTimestamp = 0.0;
    }

    public void updateInputs(LimelightIOInputs inputs) {
    }
}
