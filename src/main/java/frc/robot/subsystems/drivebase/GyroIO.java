package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yaw = new Rotation2d();
    }

    public void updateInputs(GyroIOInputs inputs) {
    }

    public void resetYaw() {
    }
}
