package frc.robot;

import frc.robot.constants.GeneralConstants;

import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

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

    public static Pose2d flipIfNeeded(Pose2d pose) {
        if (shouldFlip()) {
            return GeometryUtil.flipFieldPose(pose);
        } else {
            return pose;
        }
    }
}
