package frc.robot.auto

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.flipIfNeededNow
import frc.robot.make
import frc.robot.subsystems.drivebase.Drivebase
import frc.robot.subsystems.launcher.Launcher
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object AutoGenerator {
    private val messUpMidfieldTopNote = LoggedDashboardBoolean("Auto Generator: Mess with Top", true)
    private val messUpMidfieldUpperMiddleNote = LoggedDashboardBoolean("Auto Generator: Mess with Upper Middle", true)
    private val messUpMidfieldMiddleNote = LoggedDashboardBoolean("Auto Generator: Mess with Middle", true)
    private val messUpMidfieldLowerMiddleNote = LoggedDashboardBoolean("Auto Generator: Mess with Lower Middle", true)
    private val messUpMidfieldBottomNote = LoggedDashboardBoolean("Auto Generator: Mess with Bottom", true)

    private val startingPath = make {
        val startingPoints = LoggedDashboardChooser<PathPlannerPath>("Auto Generator: Starting Point")
        startingPoints.addDefaultOption("Top", PathPlannerPath.fromPathFile("Left starting point to subwoofer"))
        startingPoints.addOption("Middle", PathPlannerPath.fromPathFile("Middle starting point to subwoofer"))
        startingPoints.addOption("Bottom", PathPlannerPath.fromPathFile("Right starting point to subwoofer"))
        startingPoints
    }


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
    fun generateAuto(): Command {
        return Commands.deferredProxy { generateAutoNow() }
    }

    private fun generateAutoNow(): Command {
        val commands = SequentialCommandGroup()

        commands.addCommands(Drivebase.setPoseCommand(flipIfNeededNow(startingPath.get().startingDifferentialPose)))
        commands.addCommands(AutoBuilder.followPath(startingPath.get()))
        commands.addCommands(Launcher.launchCommand())

        var path: PathPlannerPath? = null
        path = if (messUpMidfieldTopNote.get()) {
            PathPlannerPath.fromPathFile("Mess with far left midfield note")
        } else {
            PathPlannerPath.fromPathFile("Go near far left midfield")
        }
        commands.addCommands(AutoBuilder.followPath(path))

        val rotation = Rotation2d.fromDegrees(-90.0)

        val bezierPoints = PathPlannerPath.bezierFromPoses(
            flipIfNeededNow(
                if (messUpMidfieldTopNote.get()) Pose2d(8.30, 7.40, rotation) else Pose2d(
                    7.30,
                    7.40,
                    rotation
                )
            ),
            flipIfNeededNow(
                if (messUpMidfieldUpperMiddleNote.get()) Pose2d(8.30, 5.75, rotation) else Pose2d(
                    7.30,
                    5.75,
                    rotation
                )
            ),
            flipIfNeededNow(
                if (messUpMidfieldMiddleNote.get()) Pose2d(8.30, 4.10, rotation) else Pose2d(
                    7.30,
                    4.10,
                    rotation
                )
            ),
            flipIfNeededNow(
                if (messUpMidfieldLowerMiddleNote.get()) Pose2d(8.30, 2.50, rotation) else Pose2d(
                    7.30,
                    2.50,
                    rotation
                )
            ),
            flipIfNeededNow(
                if (messUpMidfieldBottomNote.get()) Pose2d(8.30, 0.80, rotation) else Pose2d(
                    7.30,
                    0.80,
                    rotation
                )
            )
        )

        path = PathPlannerPath(
            bezierPoints,
            PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
            GoalEndState(0.0, Rotation2d.fromDegrees(90.0))
        )

        path.preventFlipping = true
        commands.addCommands(AutoBuilder.followPath(path))

        return commands
    }
}
