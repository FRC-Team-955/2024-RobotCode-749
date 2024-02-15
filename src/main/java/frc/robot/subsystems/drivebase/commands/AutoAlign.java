package frc.robot.subsystems.drivebase.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util;
import frc.robot.commands.Controller;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.util.Rect2d;

public class AutoAlign {
    private final Drivebase drivebase;

    public AutoAlign(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

    private Command autoAlignCommand(Supplier<Pose2d> targetPose, Rect2d bounds, CommandXboxController errorController) {
        return Commands.either(
                drivebase.pathfindCommand(targetPose),
                Controller.setRumbleError(errorController),
                () -> Util.flipIfNeededNow(bounds).contains(drivebase.getPose())
        );
    }

    public Command rightSubwooferCommand(CommandXboxController errorController) {
        var targetPose = Util.flipIfNeeded(new Pose2d(1.185, 6.612, Rotation2d.fromRadians(0.696)));
        var bounds = new Rect2d(
                new Pose2d(1.203, 6.261, new Rotation2d()),
                new Pose2d(5.339, 7.737, new Rotation2d())
        );
        return autoAlignCommand(targetPose, bounds, errorController);
    }

    public Command leftSubwooferCommand(CommandXboxController errorController) {
        var targetPose = Util.flipIfNeeded(new Pose2d(1.195, 4.545, Rotation2d.fromRadians(-1.106)));
        var bounds = new Rect2d(
                new Pose2d(1.373, 5.101, new Rotation2d()),
                new Pose2d(2.632, 1.354, new Rotation2d())
        );
        return autoAlignCommand(targetPose, bounds, errorController);
    }

    public Command frontSubwooferCommand(CommandXboxController errorController) {
        var targetPose = Util.flipIfNeeded(new Pose2d(1.391, 5.55, Rotation2d.fromRadians(Math.PI)));
        var bounds = new Rect2d(
            new Pose2d(1.469, 1.6, new Rotation2d()),
            new Pose2d(2.626, 7.783, new Rotation2d())
        );
        return autoAlignCommand(targetPose, bounds, errorController);
    }

    public Command sourceCommand(CommandXboxController errorController) {
        var targetPose = Util.flipIfNeeded(new Pose2d(1.93, 7.716, Rotation2d.fromDegrees(90)));
        var bounds = new Rect2d(
                new Pose2d(1.272, 1.455, new Rotation2d()),
                new Pose2d(2.798, 7.681, new Rotation2d())
        );
        return autoAlignCommand(targetPose, bounds, errorController);
    }
}
