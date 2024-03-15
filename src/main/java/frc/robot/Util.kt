@file:JvmName("Util")

package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.util.Rect2d
import org.littletonrobotics.junction.AutoLogOutputManager
import java.util.function.Supplier

fun <T> switchMode(real: Supplier<T>, sim: Supplier<T>, replay: Supplier<T>): T {
    return when (Constants.mode) {
        Constants.Mode.REAL -> {
            real.get()
        }

        Constants.Mode.SIM -> {
            sim.get()
        }

        else -> {
            // Can't be case REPLAY because of Java restrictions
            replay.get()
        }
    }
}

fun <T> ifSimElse(sim: T, realAndReplay: T): T {
    return if (Constants.mode == Constants.Mode.SIM) sim
    else realAndReplay
}

fun <T> ifRealElse(real: Supplier<T>, simAndReplay: Supplier<T>): T {
    return if (Constants.mode == Constants.Mode.REAL) real.get()
    else simAndReplay.get()
}

fun <T> make(maker: Supplier<T>): T {
    return maker.get()
}

fun shouldFlip(): Boolean {
    return DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
}

fun flipIfNeeded(pose: Pose2d): Supplier<Pose2d> {
    return Supplier { flipIfNeededNow(pose) }
}

fun flipIfNeeded(pose: Rect2d): Supplier<Rect2d> {
    return Supplier { flipIfNeededNow(pose) }
}

fun flipIfNeededNow(pose: Pose2d): Pose2d {
    return if (shouldFlip()) {
        GeometryUtil.flipFieldPose(pose)
    } else {
        pose
    }
}

fun flipIfNeededNow(rect: Rect2d): Rect2d {
    if (shouldFlip()) {
        val blX = rect.bottomLeftCorner.x
        val blY = rect.bottomLeftCorner.y
        val trX = rect.topRightCorner.x
        val trY = rect.topRightCorner.y
        // when flipping a Rect2d, we need to swap the X positions otherwise it will be an inside out rectangle and therefore .contains will always return false
        return Rect2d(
            GeometryUtil.flipFieldPose(Pose2d(trX, blY, rect.bottomLeftCorner.rotation)),
            GeometryUtil.flipFieldPose(Pose2d(blX, trY, rect.topRightCorner.rotation))
        )
    } else {
        return rect
    }
}

fun buildAllianceAuto(name: String): Command {
    val blue = AutoBuilder.buildAuto("B_$name")
    val red = AutoBuilder.buildAuto("R_$name")
    return Commands.deferredProxy { if (shouldFlip()) red else blue }
}

fun registerFieldsForAutoLogOutput(vararg roots: Any) {
    roots.forEach { roots ->
        val method = AutoLogOutputManager::class.java.getDeclaredMethod("registerFields", Object::class.java)
        method.isAccessible = true
        method.invoke(null, roots)
    }
}
