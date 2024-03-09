package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.GeneralConstants;
import frc.robot.util.Rect2d;

import java.io.File;
import java.util.function.Supplier;

public final class Util {
    public static <T> T switchMode(Supplier<T> real, Supplier<T> sim, Supplier<T> replay) {
        switch (GeneralConstants.mode) {
            case REAL -> {
                return real.get();
            }
            case SIM -> {
                return sim.get();
            }
            default -> { // Can't be case REPLAY because of Java restrictions
                return replay.get();
            }
        }
    }

    public static <T> T ifSimElse(T sim, T realAndReplay) {
        if (GeneralConstants.mode == GeneralConstants.Mode.SIM) return sim;
        else return realAndReplay;
    }

    public static <T> T ifRealElse(Supplier<T> real, Supplier<T> simAndReplay) {
        if (GeneralConstants.mode == GeneralConstants.Mode.REAL) return real.get();
        else return simAndReplay.get();
    }

    public static <T> T make(Supplier<T> maker) {
        return maker.get();
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Supplier<Pose2d> flipIfNeeded(Pose2d pose) {
        return () -> flipIfNeededNow(pose);
    }

    public static Supplier<Rect2d> flipIfNeeded(Rect2d pose) {
        return () -> flipIfNeededNow(pose);
    }

    public static Pose2d flipIfNeededNow(Pose2d pose) {
        if (shouldFlip()) {
            return GeometryUtil.flipFieldPose(pose);
        } else {
            return pose;
        }
    }

    public static Rect2d flipIfNeededNow(Rect2d rect) {
        if (shouldFlip()) {
            var blX = rect.bottomLeftCorner.getX();
            var blY = rect.bottomLeftCorner.getY();
            var trX = rect.topRightCorner.getX();
            var trY = rect.topRightCorner.getY();
            // when flipping a Rect2d, we need to swap the X positions otherwise it will be an inside out rectangle and therefore .contains will always return false
            return new Rect2d(GeometryUtil.flipFieldPose(new Pose2d(trX, blY, rect.bottomLeftCorner.getRotation())), GeometryUtil.flipFieldPose(new Pose2d(blX, trY, rect.topRightCorner.getRotation())));
        } else {
            return rect;
        }
    }

    public static boolean fileConstant(String fileName, boolean fallback) {
        if (!GeneralConstants.useFileConstants) return fallback;
        return new File(Filesystem.getDeployDirectory(), "config/" + fileName).exists();
    }

    public static double speed(CommandXboxController controller) {
        return -controller.getLeftTriggerAxis() + controller.getRightTriggerAxis();
    }

    public static Command buildAllianceAuto(String name) {
        var blue = AutoBuilder.buildAuto("B_" + name);
        var red = AutoBuilder.buildAuto("R_" + name);
        return Commands.deferredProxy(() -> shouldFlip() ? red : blue);
    }
}
