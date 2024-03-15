package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d

class Rect2d(var bottomLeftCorner: Pose2d, var topRightCorner: Pose2d) {
    fun contains(p: Pose2d): Boolean {
        return (
                p.x > bottomLeftCorner.x
                        && p.y > bottomLeftCorner.y
                        && p.x < topRightCorner.x
                        && p.y < topRightCorner.y
                )
    }
}
