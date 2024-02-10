package frc.robot.auto;

import java.util.List;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util;
import frc.robot.subsystems.drivebase.Drivebase;

public class AutoGenerator {
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Auto Generator");
    private static final GenericEntry messUpMidfieldTopNote         = tab.add("Mess Top", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static final GenericEntry messUpMidfieldUpperMiddleNote = tab.add("Mess UpperMiddle", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static final GenericEntry messUpMidfieldMiddleNote      = tab.add("Mess Middle", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static final GenericEntry messUpMidfieldLowerMiddleNote = tab.add("Mess LowerMiddle", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static final GenericEntry messUpMidfieldBottomNote      = tab.add("Mess Bottom", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    private static final SendableChooser<StartingPoint> startingPoints = Util.make(() -> {
        var startingPoints = new SendableChooser<StartingPoint>();
        startingPoints.setDefaultOption("Top", StartingPoint.Top);
        startingPoints.addOption("Middle", StartingPoint.Middle);
        startingPoints.addOption("Bottom", StartingPoint.Bottom);
        return startingPoints;
    });

    private static final SendableChooser<Note> notes = Util.make(() -> {
        var notes = new SendableChooser<Note>();
        notes.setDefaultOption("Wing Top", Note.WingTopNote);
        notes.addOption("Wing Middle", Note.WingMiddleNote);
        notes.addOption("Wing Bottom", Note.WingBottomNote);
        notes.addOption("Midfield Top", Note.MidfieldTopNote);
        notes.addOption("Midfield Upper Middle", Note.MidfieldUpperMiddleNote);
        notes.addOption("Midfield Middle", Note.MidfieldMiddleNote);
        notes.addOption("Midfield Lower Middle", Note.MidfieldLowerMiddleNote);
        notes.addOption("Midfield Bottom", Note.MidfieldBottomNote);
        return notes;
    });

    private static Command followPath(String pathName) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    }

    public static Command generateAuto(Drivebase drivebase){
        var startingPoint = startingPoints.getSelected();
        // var note = AutoGenerator.notes.getSelected();
        var commands = new SequentialCommandGroup();

        switch (startingPoint) {
            case Top -> {
                var path = PathPlannerPath.fromPathFile("Starting top to subwoofer");
                commands.addCommands(drivebase.setPoseCommand(GeometryUtil.flipFieldPose(path.getStartingDifferentialPose())));
                commands.addCommands(AutoBuilder.followPath(path));
            }
            case Middle -> {
                var path = PathPlannerPath.fromPathFile("Starting middle to subwoofer");
                commands.addCommands(drivebase.setPoseCommand(GeometryUtil.flipFieldPose(path.getStartingDifferentialPose())));
                commands.addCommands(AutoBuilder.followPath(path));
            }
            case Bottom -> {
                var path = PathPlannerPath.fromPathFile("Starting bottom to subwoofer");
                commands.addCommands(drivebase.setPoseCommand(GeometryUtil.flipFieldPose(path.getStartingDifferentialPose())));
                commands.addCommands(AutoBuilder.followPath(path));
            }
        }

        // public void rickroll() {
            // We're no strangers to love
            // You know the rules and so do I (do I)
            // A full commitment's what I'm thinking of
            // You wouldn't get this from any other guy
            // I just wanna tell you how I'm feeling
            // Gotta make you understand
            // Never gonna give you up
            // Never gonna let you down
            // Never gonna run around and desert you
            // Never gonna make you cry
            // Never gonna say goodbye
            // Never gonna tell a lie and hurt you
            // We've known each other for so long
            // Your heart's been aching, but you're too shy to say it (say it)
            // Inside, we both know what's been going on (going on)
            // We know the game and we're gonna play it
            // And if you ask me how I'm feeling
            // Don't tell me you're too blind to see
            // Never gonna give you up
            // Never gonna let you down
            // Never gonna run around and desert you
            // Never gonna make you cry
            // Never gonna say goodbye
            // Never gonna tell a lie and hurt you
            // Never gonna give you up
            // Never gonna let you down
            // Never gonna run around and desert you
            // Never gonna make you cry
            // Never gonna say goodbye
            // Never gonna tell a lie and hurt you
            // We've known each other for so long
            // Your heart's been aching, but you're too shy to say it (to say it)
            // Inside, we both know what's been going on (going on)
            // We know the game and we're gonna play it
            // I just wanna tell you how I'm feeling
            // Gotta make you understand
            // Never gonna give you up
            // Never gonna let you down
            // Never gonna run around and desert you
            // Never gonna make you cry
            // Never gonna say goodbye
            // Never gonna tell a lie and hurt you
            // Never gonna give you up
            // Never gonna let you down
            // Never gonna run around and desert you
            // Never gonna make you cry
            // Never gonna say goodbye
            // Never gonna tell a lie and hurt you
            // Never gonna give you up
            // Never gonna let you down
            // Never gonna run around and desert you
            // Never gonna make you cry
            // Never gonna say goodbye
            // Never gonna tell a lie and hurt you
        // }

        Rotation2d rotation = null;

        switch (startingPoint) {
            case Top: rotation = new Rotation2d(90); break;
            case Middle: rotation = new Rotation2d(90); break;
            case Bottom: rotation = new Rotation2d(-90); break;
            default: rotation = new Rotation2d(90); break;
        }

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            drivebase.getPose(),
            Util.flipIfNeeded(messUpMidfieldTopNote.getBoolean(true)         ? new Pose2d(7.80, 7.40, rotation) : new Pose2d(7.30, 7.40, rotation)),
            Util.flipIfNeeded(messUpMidfieldUpperMiddleNote.getBoolean(true) ? new Pose2d(7.80, 5.75, rotation) : new Pose2d(7.30, 5.75, rotation))
            // Util.flipIfNeeded(messUpMidfieldMiddleNote.getBoolean(true)      ? new Pose2d(7.80, 4.10, rotation) : new Pose2d(7.30, 4.10, rotation))
            // messUpMidfieldLowerMiddleNote.getBoolean(true) ? Util.flipIfNeeded(new Pose2d(7.80, 2.50, rotation)) : Util.flipIfNeeded(new Pose2d(7.30, 2.50, rotation)),
            // messUpMidfieldBottomNote.getBoolean(true)      ? Util.flipIfNeeded(new Pose2d(7.80, 0.80, rotation)) : Util.flipIfNeeded(new Pose2d(7.30, 0.80, rotation))
        );

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(90)));
        
        path.preventFlipping = true;
        commands.addCommands(AutoBuilder.followPath(path));

        //     switch (note) {
            //         case WingTopNote -> {
            //             commands.addCommands(
            //                 followPath("Subwoofer to top note"),
            //                 followPath("Top note to subwoofer")
            //                 );
            //         }
            //         case WingMiddleNote -> {
            //             commands.addCommands(
            //                 followPath("Subwoofer to middle note"),
            //                 followPath("Middle note to subwoofer")
            //             );
            //         }
            //         case WingBottomNote -> {
            //             commands.addCommands(
            //                 followPath("Subwoofer to bottom note"),
            //                 followPath("Bottom note to subwoofer")
            //             );
            //         }
            //         case MidfieldTopNote -> {
            //             commands.addCommands(
            //                 followPath("Subwoofer to midfield(1) note"),
            //                 followPath("Midfield(1) to subwoofer")
            //             );
            //         }
            //         case MidfieldUpperMiddleNote -> {
            //             commands.addCommands(
            //                 followPath("Subwoofer to midfield(2) note"),
            //                 followPath("Midfield(2) to subwoofer")
            //             );
            //         }
            //         case MidfieldMiddleNote -> {
            //             commands.addCommands(
            //                 followPath("Subwoofer to midfield(3) note"),
            //                 followPath("Midfield(3) to subwoofer")
            //             );
            //         }
            //         case MidfieldLowerMiddleNote -> {
            //             commands.addCommands(
            //                 followPath("Subwoofer to midfield(4) note"),
            //                 followPath("Midfield(4) to subwoofer")
            //             );
            //         }
            //         case MidfieldBottomNote -> {
            //             commands.addCommands(
            //                 followPath("Subwoofer to midfield(5) note"),
            //                 followPath("Midfield(5) to subwoofer")
            //             );
            //         }
        //     }

        return commands;
    }

    public static void initializeShuffleboard() {
        tab.add("Starting Point", startingPoints);

        tab.add("Notes", notes);
    }
}
