package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.util.TunablePIDController;

public class DrivebaseIOSim extends DrivebaseIO {
    private static final double P = 0.05;
    private static final double D = 0.001;

    private final DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
            DrivebaseConstants.motor,
            DrivebaseConstants.gearRatio,
            DrivebaseConstants.wheelSize,
            null
    );

    private double leftAppliedVolts = 0;
    private double rightAppliedVolts = 0;
    private boolean closedLoop = false;
    private final TunablePIDController leftPID = new TunablePIDController("Drivebase Left (Sim)", P, 0, D);
    private final TunablePIDController rightPID = new TunablePIDController("Drivebase Right (Sim)", P, 0, D);
    private double leftFFVolts = 0;
    private double rightFFVolts = 0;

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
        if (closedLoop) {
            leftAppliedVolts = MathUtil.clamp(
                    leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / DrivebaseConstants.wheelRadius) + leftFFVolts,
                    -12.0,
                    12.0
            );
            rightAppliedVolts = MathUtil.clamp(
                    leftPID.calculate(sim.getRightVelocityMetersPerSecond() / DrivebaseConstants.wheelRadius) + rightFFVolts,
                    -12.0,
                    12.0
            );
            sim.setInputs(leftAppliedVolts, rightAppliedVolts);
        }

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
        closedLoop = false;
        leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
        rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
        sim.setInputs(leftAppliedVolts, rightAppliedVolts);
    }

    @Override
    public void setVelocity(
            double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
        closedLoop = true;
        leftPID.setSetpoint(leftRadPerSec);
        rightPID.setSetpoint(rightRadPerSec);
        this.leftFFVolts = leftFFVolts;
        this.rightFFVolts = rightFFVolts;
    }
}
