package frc.robot.subsystems.drivebase

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer

class LimelightIOReal : LimelightIO() {
    override fun updateInputs(inputs: LimelightIOInputs) {
        val left = NetworkTableInstance.getDefault().getTable("limelight-left")
        inputs.leftTv = left.getEntry("tv").getInteger(-1)
        val leftBotpose = left.getEntry("botpose_wpiblue").getDoubleArray(null as DoubleArray?)
        if (leftBotpose != null && leftBotpose.size >= 11) {
            inputs.leftBotpose = Pose2d(leftBotpose[0], leftBotpose[1], Rotation2d.fromDegrees(leftBotpose[5]))
            inputs.leftBotposeTimestamp = Timer.getFPGATimestamp() - (leftBotpose[6] / 1000.0)
            inputs.leftTagCount = leftBotpose[7]
            inputs.leftAvgArea = leftBotpose[10]
        } else {
            inputs.leftTv = -2
            inputs.leftBotpose = noBotpose()
        }

        val right = NetworkTableInstance.getDefault().getTable("limelight-right")
        inputs.rightTv = right.getEntry("tv").getInteger(-1)
        val rightBotpose = right.getEntry("botpose_wpiblue").getDoubleArray(null as DoubleArray?)
        if (rightBotpose != null && rightBotpose.size >= 11) {
            inputs.rightBotpose = Pose2d(rightBotpose[0], rightBotpose[1], Rotation2d.fromDegrees(rightBotpose[5]))
            inputs.rightBotposeTimestamp = Timer.getFPGATimestamp() - (rightBotpose[6] / 1000.0)
            inputs.rightTagCount = rightBotpose[7]
            inputs.rightAvgArea = rightBotpose[10]
        } else {
            inputs.rightTv = -2
            inputs.rightBotpose = noBotpose()
        }
    }
}
