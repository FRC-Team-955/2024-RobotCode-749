package frc.robot.subsystems.drivebase;


public class GyroIOSim extends GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = DrivebaseIOSim.getHeading();
    }
}
