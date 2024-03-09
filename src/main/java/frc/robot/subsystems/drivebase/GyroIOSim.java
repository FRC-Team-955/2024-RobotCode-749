package frc.robot.subsystems.drivebase;


import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim extends GyroIO {
    private Rotation2d rotationOffset = new Rotation2d();

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = DrivebaseIOSim.getHeading().minus(rotationOffset);
    }

    @Override
    public void setYaw(double degrees) {
        rotationOffset = DrivebaseIOSim.getHeading().plus(Rotation2d.fromDegrees(degrees));
    }
}
