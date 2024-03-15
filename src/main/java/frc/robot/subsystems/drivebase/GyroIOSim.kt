package frc.robot.subsystems.drivebase

import edu.wpi.first.math.geometry.Rotation2d


class GyroIOSim : GyroIO() {
    private var rotationOffset = Rotation2d()

    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.yaw = DrivebaseIOSim.heading.minus(rotationOffset)
    }

    override fun setYaw(yaw: Rotation2d) {
        rotationOffset = DrivebaseIOSim.heading.minus(yaw)
    }
}
