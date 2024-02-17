package frc.robot.subsystems.drivebase.commands;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.util.Rect2d;

import java.util.Optional;
import java.util.function.Supplier;

public class AutoAlign {
    private final Drivebase drivebase;

    private static final Pose2d rightSubwoofer = new Pose2d(1.195, 4.4, Rotation2d.fromRadians(3.0));
    private static final Pose2d leftSubwoofer = new Pose2d(1.3, 6.2, Rotation2d.fromRadians(-3.1));
    private static final Pose2d frontSubwoofer = new Pose2d(1.391, 5.55, Rotation2d.fromRadians(/* 180 degrees */ Math.PI));
    private static final Pose2d source = new Pose2d(15.38, 0.958, Rotation2d.fromRadians(-1));

    private static final Rect2d[] subwooferBounds = new Rect2d[]{
            new Rect2d(
                    new Pose2d(1.4, 4.6, new Rotation2d()),
                    new Pose2d(4.7, 7.3, new Rotation2d())
            ),
            new Rect2d(
                    new Pose2d(0.7, 1.2, new Rotation2d()),
                    new Pose2d(2.8, 4.6, new Rotation2d())
            )
    };

    public AutoAlign(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

    public Optional<Command> rightSubwooferCommand() {
        // Need to swap right and left on red
        Supplier<Pose2d> targetPose = () -> Util.shouldFlip() ? GeometryUtil.flipFieldPose(leftSubwoofer) : rightSubwoofer;
        return getAutoAlignCommand(targetPose, subwooferBounds);
    }

    public Optional<Command> leftSubwooferCommand() {
        // Need to swap right and left on red
        Supplier<Pose2d> targetPose = () -> Util.shouldFlip() ? GeometryUtil.flipFieldPose(rightSubwoofer) : leftSubwoofer;
        return getAutoAlignCommand(targetPose, subwooferBounds);
    }

    public Optional<Command> frontSubwooferCommand() {
        var targetPose = Util.flipIfNeeded(frontSubwoofer);
        return getAutoAlignCommand(targetPose, subwooferBounds);
    }

    public Optional<Command> sourceCommand() {
        var targetPose = Util.flipIfNeeded(source);
        var bounds = new Rect2d(
                new Pose2d(11.9, 0.4, new Rotation2d()),
                new Pose2d(16.3, 4.2, new Rotation2d())
        );
        return getAutoAlignCommand(targetPose, bounds);
    }

    private Optional<Command> getAutoAlignCommand(Supplier<Pose2d> targetPose, Rect2d... bounds) {
        for (var bound : bounds) {
            if (Util.flipIfNeededNow(bound).contains(drivebase.getPose()))
                return Optional.of(drivebase.pathfindCommand(targetPose));
        }
        return Optional.empty();
    }
}
