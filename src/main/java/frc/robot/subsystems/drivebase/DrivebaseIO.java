package frc.robot.subsystems.drivebase;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DrivebaseIO {
    public static DrivebaseIOInputs inputs() {
        // Kotlin compiles before Java, so the generated class doesn't always exist if we use it in Kotlin code. We also have to make the inputs class abstract and implement LoggableInputs, otherwise kotlin will complain when we try to call Logger.processInputs()
        return new DrivebaseIOInputsAutoLogged();
    }

    @AutoLog // @AutoLog doesn't work with Kotlin classes, so this file has to be Java
    public static abstract class DrivebaseIOInputs implements LoggableInputs {
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
