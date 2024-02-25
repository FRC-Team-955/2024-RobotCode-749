package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.launcher.Launcher;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

public class AutoGenerator {
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Auto Generator");
    private static final GenericEntry messUpMidfieldTopNote = tab.add("Mess with Top", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static final GenericEntry messUpMidfieldUpperMiddleNote = tab.add("Mess with Upper Middle", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static final GenericEntry messUpMidfieldMiddleNote = tab.add("Mess with Middle", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static final GenericEntry messUpMidfieldLowerMiddleNote = tab.add("Mess with Lower Middle", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static final GenericEntry messUpMidfieldBottomNote = tab.add("Mess with Bottom", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    private static final LoggedDashboardChooser<StartingPoint> startingPoints = Util.make(() -> {
        var startingPoints = new LoggedDashboardChooser<StartingPoint>("Starting Point");
        startingPoints.addDefaultOption("Top", StartingPoint.Top);
        startingPoints.addOption("Middle", StartingPoint.Middle);
        startingPoints.addOption("Bottom", StartingPoint.Bottom);
        return startingPoints;
    });

    // private static final SendableChooser<Note> notes = Util.make(() -> {
    //     var notes = new SendableChooser<Note>();
    //     notes.setDefaultOption("Wing Top", Note.WingTopNote);
    //     notes.addOption("Wing Middle", Note.WingMiddleNote);
    //     notes.addOption("Wing Bottom", Note.WingBottomNote);
    //     notes.addOption("Midfield Top", Note.MidfieldTopNote);
    //     notes.addOption("Midfield Upper Middle", Note.MidfieldUpperMiddleNote);
    //     notes.addOption("Midfield Middle", Note.MidfieldMiddleNote);
    //     notes.addOption("Midfield Lower Middle", Note.MidfieldLowerMiddleNote);
    //     notes.addOption("Midfield Bottom", Note.MidfieldBottomNote);
    //     return notes;
    // });

    // private static Command followPath(String pathName) {
    //     return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    // }

    public static Command generateAuto(Drivebase drivebase, Launcher launcher) {
        var startingPoint = startingPoints.get();
        var commands = new SequentialCommandGroup();

        PathPlannerPath path = null;
        switch (startingPoint) {
            case Top -> path = PathPlannerPath.fromPathFile("Left starting point to subwoofer");
            case Middle -> path = PathPlannerPath.fromPathFile("Middle starting point to subwoofer");
            case Bottom -> path = PathPlannerPath.fromPathFile("Right starting point to subwoofer");
        }
        commands.addCommands(drivebase.setPoseCommand(GeometryUtil.flipFieldPose(path.getStartingDifferentialPose())));
        commands.addCommands(AutoBuilder.followPath(path));
        commands.addCommands(launcher.launchCommand());

        if (messUpMidfieldTopNote.getBoolean(true)) {
            path = PathPlannerPath.fromPathFile("Mess with far left midfield note");
        } else {
            path = PathPlannerPath.fromPathFile("Go near far left midfield");
        }
        commands.addCommands(AutoBuilder.followPath(path));

        Rotation2d rotation = Rotation2d.fromDegrees(-90);

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                Util.flipIfNeededNow(messUpMidfieldTopNote.getBoolean(true) ? new Pose2d(7.70, 7.40, rotation) : new Pose2d(7.30, 7.40, rotation)),
                Util.flipIfNeededNow(messUpMidfieldUpperMiddleNote.getBoolean(true) ? new Pose2d(7.70, 5.75, rotation) : new Pose2d(7.30, 5.75, rotation)),
                Util.flipIfNeededNow(messUpMidfieldMiddleNote.getBoolean(true) ? new Pose2d(7.70, 4.10, rotation) : new Pose2d(7.30, 4.10, rotation)),
                Util.flipIfNeededNow(messUpMidfieldLowerMiddleNote.getBoolean(true) ? new Pose2d(7.70, 2.50, rotation) : new Pose2d(7.30, 2.50, rotation)),
                Util.flipIfNeededNow(messUpMidfieldBottomNote.getBoolean(true) ? new Pose2d(7.70, 0.80, rotation) : new Pose2d(7.30, 0.80, rotation))
        );

        path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
                new GoalEndState(0.0, Rotation2d.fromDegrees(90))
        );

        path.preventFlipping = true;
        commands.addCommands(AutoBuilder.followPath(path));

        return commands;
    }

    public static void initializeShuffleboard() {
        tab.add("Starting Point", startingPoints);
        // tab.add("Notes", notes);
    }
}
