package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.GeneralConstants;
import frc.robot.util.Rect2d;

public final class Util {
    public static <T> T chooseIO(Supplier<T> real, Supplier<T> sim, Supplier<T> replay) {
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
            return new Rect2d(GeometryUtil.flipFieldPose(new Pose2d(trX, blY, new Rotation2d())), GeometryUtil.flipFieldPose(new Pose2d(blX, trY, new Rotation2d())));
        } else {
            return rect;
        }
    }

    public static boolean fileConstant(String fileName, boolean fallback) {
        if (!GeneralConstants.useFileConstants) return fallback;
        return new File(Filesystem.getDeployDirectory(), "config/" + fileName).exists();
    }
}
