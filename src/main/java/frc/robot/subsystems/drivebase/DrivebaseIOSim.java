package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants;

public class DrivebaseIOSim extends DrivebaseIO {
    private static final DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
            Constants.Drivebase.motor,
            Constants.Drivebase.gearRatio,
            Constants.Drivebase.wheelSize,
            null
    );

    private double leftAppliedVolts = 0;
    private double rightAppliedVolts = 0;

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
        sim.update(0.02);
        inputs.leftPositionRad = sim.getLeftPositionMeters() / Constants.Drivebase.wheelRadius;
        inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / Constants.Drivebase.wheelRadius;
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftLeaderCurrentAmps = sim.getLeftCurrentDrawAmps();
        inputs.leftFollowerCurrentAmps = sim.getLeftCurrentDrawAmps();

        inputs.rightPositionRad = sim.getRightPositionMeters() / Constants.Drivebase.wheelRadius;
        inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / Constants.Drivebase.wheelRadius;
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightLeaderCurrentAmps = sim.getRightCurrentDrawAmps();
        inputs.rightFollowerCurrentAmps = sim.getRightCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
        rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
        sim.setInputs(leftAppliedVolts, rightAppliedVolts);
    }

    public static Rotation2d getHeading() {
        return sim.getHeading();
    }

    public static void resetHeading() {
        var pose = sim.getPose();
        sim.setPose(new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
    }
}
