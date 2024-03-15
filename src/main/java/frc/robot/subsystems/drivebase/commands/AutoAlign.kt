package frc.robot.subsystems.drivebase.commands

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.flipIfNeeded
import frc.robot.flipIfNeededNow
import frc.robot.shouldFlip
import frc.robot.subsystems.drivebase.Drivebase
import frc.robot.util.Rect2d
import java.util.*
import java.util.function.Supplier

object AutoAlign {
    fun rightSubwooferCommand(boundsCheck: Boolean): Optional<Command> {
        // Need to swap right and left on red
        val targetPose = Supplier { if (shouldFlip()) GeometryUtil.flipFieldPose(leftSubwoofer) else rightSubwoofer }
        return getAutoAlignCommand(
            targetPose,
            boundsCheck,
            *subwooferBounds
        ).map { c: Command -> c.withName("AutoAlign\$rightSubwoofer") }
    }

    fun leftSubwooferCommand(boundsCheck: Boolean): Optional<Command> {
        // Need to swap right and left on red
        val targetPose = Supplier { if (shouldFlip()) GeometryUtil.flipFieldPose(rightSubwoofer) else leftSubwoofer }
        return getAutoAlignCommand(
            targetPose,
            boundsCheck,
            *subwooferBounds
        ).map { c: Command -> c.withName("AutoAlign\$leftSubwoofer") }
    }

    fun frontSubwooferCommand(boundsCheck: Boolean): Optional<Command> {
        val targetPose = flipIfNeeded(frontSubwoofer)
        return getAutoAlignCommand(
            targetPose,
            boundsCheck,
            *subwooferBounds
        ).map { c: Command -> c.withName("AutoAlign\$frontSubwoofer") }
    }

    fun sourceCommand(boundsCheck: Boolean): Optional<Command> {
        val targetPose = flipIfNeeded(source)
        val bounds = Rect2d(
            Pose2d(10.9, 0.2, Rotation2d()),
            Pose2d(16.3, 4.2, Rotation2d())
        )
        return getAutoAlignCommand(
            targetPose,
            boundsCheck,
            bounds
        ).map { c: Command -> c.withName("AutoAlign\$source") }
    }

    private fun getAutoAlignCommand(
        targetPose: Supplier<Pose2d>,
        boundsCheck: Boolean,
        vararg bounds: Rect2d
    ): Optional<Command> {
        if (!boundsCheck) return Optional.of(Drivebase.pathfindCommand(targetPose))
        for (bound in bounds) {
            if (flipIfNeededNow(bound).contains(Drivebase.pose)) return Optional.of(
                Drivebase.pathfindCommand(targetPose)
            )
        }
        return Optional.empty()
    }

    private val rightSubwoofer = Pose2d(1.195, 4.4, Rotation2d.fromRadians(3.0))
    private val leftSubwoofer = Pose2d(1.3, 6.2, Rotation2d.fromRadians(-3.1))
    private val frontSubwoofer = Pose2d(1.391, 6.0, Rotation2d.fromRadians(/* 180 degrees */ Math.PI))
    private val source = Pose2d(15.38, 0.958, Rotation2d.fromRadians(-1.0))

    private val subwooferBounds = arrayOf(
        // Top
        Rect2d(
            Pose2d(1.0, 4.6, Rotation2d()),
            Pose2d(6.0, 7.7, Rotation2d())
        ),
        // Bottom
        Rect2d(
            Pose2d(0.1, 0.2, Rotation2d()),
            Pose2d(2.8, 4.6, Rotation2d())
        )
    )
}
