package frc.robot.subsystems.drivebase.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import frc.robot.subsystems.drivebase.Drivebase;

public class AutoAlign {
    private final Drivebase drivebase;

    public AutoAlign(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

    public Command rightSubwooferCommand() {
        return drivebase.pathfindCommand(Util.flipIfNeeded(new Pose2d(1.185, 6.612, Rotation2d.fromRadians(0.696))));
    }

    public Command leftSubwooferCommand() {
        return drivebase.pathfindCommand(Util.flipIfNeeded(new Pose2d(1.195, 4.545, Rotation2d.fromRadians(-1.106))));
    }

    public Command intakeSubwooferCommand() {
        return drivebase.pathfindCommand(Util.flipIfNeeded(new Pose2d(1.93, 7.716, Rotation2d.fromDegrees(90))));
    }
}
