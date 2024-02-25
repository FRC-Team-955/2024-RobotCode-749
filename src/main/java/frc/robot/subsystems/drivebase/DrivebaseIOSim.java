package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.constants.DrivebaseConstants;

public class DrivebaseIOSim extends DrivebaseIO {
    private final DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
            DrivebaseConstants.motor,
            DrivebaseConstants.gearRatio,
            DrivebaseConstants.wheelSize,
            null
    );

    private double leftAppliedVolts = 0;
    private double rightAppliedVolts = 0;

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
        sim.update(0.02);
        inputs.leftPositionRad = sim.getLeftPositionMeters() / DrivebaseConstants.wheelRadius;
        inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / DrivebaseConstants.wheelRadius;
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftLeaderCurrentAmps = sim.getLeftCurrentDrawAmps();
        inputs.leftFollowerCurrentAmps = sim.getLeftCurrentDrawAmps();

        inputs.rightPositionRad = sim.getRightPositionMeters() / DrivebaseConstants.wheelRadius;
        inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / DrivebaseConstants.wheelRadius;
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightLeaderCurrentAmps = sim.getRightCurrentDrawAmps();
        inputs.rightFollowerCurrentAmps = sim.getRightCurrentDrawAmps();

        inputs.gyroYaw = sim.getHeading();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
        rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
        sim.setInputs(leftAppliedVolts, rightAppliedVolts);
    }
}
