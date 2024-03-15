package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIO {
    public static GyroIOInputs inputs() {
        // Kotlin compiles before Java, so the generated class doesn't always exist. To make this work we also have to make the inputs class abstract and implement LoggableInputs, otherwise kotlin will complain when we try to call Logger.processInputs()
        return new GyroIOInputsAutoLogged();
    }

    @AutoLog // @AutoLog doesn't work with Kotlin classes, so this file has to be Java
    public static abstract class GyroIOInputs implements LoggableInputs {
        public boolean connected = true;
        public Rotation2d yaw = new Rotation2d();
    }

    public void updateInputs(GyroIOInputs inputs) {
    }

    public void setYaw(Rotation2d yaw) {
    }
}
