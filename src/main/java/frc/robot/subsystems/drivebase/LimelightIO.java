package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimelightIO {
    public static LimelightIOInputs inputs() {
        // Kotlin compiles before Java, so the generated class doesn't always exist if we use it in Kotlin code. We also have to make the inputs class abstract and implement LoggableInputs, otherwise kotlin will complain when we try to call Logger.processInputs()
        return new LimelightIOInputsAutoLogged();
    }

    @AutoLog // @AutoLog doesn't work with Kotlin classes, so this file has to be Java
    public static abstract class LimelightIOInputs implements LoggableInputs {
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
