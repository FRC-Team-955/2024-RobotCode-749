package frc.robot.subsystems.drivebase

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.robot.Constants

class DrivebaseIOSim : DrivebaseIO() {
    private var leftAppliedVolts = 0.0
    private var rightAppliedVolts = 0.0

    override fun updateInputs(inputs: DrivebaseIOInputs) {
        sim.update(0.02)
        inputs.leftPositionRad = sim.leftPositionMeters / Constants.Drivebase.wheelRadius
        inputs.leftVelocityRadPerSec = sim.leftVelocityMetersPerSecond / Constants.Drivebase.wheelRadius
        inputs.leftAppliedVolts = leftAppliedVolts
        inputs.leftLeaderCurrentAmps = sim.leftCurrentDrawAmps
        inputs.leftFollowerCurrentAmps = sim.leftCurrentDrawAmps

        inputs.rightPositionRad = sim.rightPositionMeters / Constants.Drivebase.wheelRadius
        inputs.rightVelocityRadPerSec = sim.rightVelocityMetersPerSecond / Constants.Drivebase.wheelRadius
        inputs.rightAppliedVolts = rightAppliedVolts
        inputs.rightLeaderCurrentAmps = sim.rightCurrentDrawAmps
        inputs.rightFollowerCurrentAmps = sim.rightCurrentDrawAmps
    }

    override fun setVoltage(leftVolts: Double, rightVolts: Double) {
        leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0)
        rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0)
        sim.setInputs(leftAppliedVolts, rightAppliedVolts)
    }

    companion object {
        private val sim: DifferentialDrivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
            Constants.Drivebase.motor,
            Constants.Drivebase.gearRatio,
            Constants.Drivebase.wheelSize,
            null
        )

        val heading: Rotation2d
            get() = sim.heading

        fun resetHeading() {
            val pose = sim.pose
            sim.pose = Pose2d(pose.x, pose.y, Rotation2d())
        }
    }
}
