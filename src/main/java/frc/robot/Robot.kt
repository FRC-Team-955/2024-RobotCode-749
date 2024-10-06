package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.auto.LaunchAndMove
import frc.robot.commands.Actions
import frc.robot.commands.SwerveMode
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.climber.LeftClimber
import frc.robot.subsystems.climber.RightClimber
import frc.robot.subsystems.controller.DriverController
import frc.robot.subsystems.controller.OperatorController
import frc.robot.subsystems.drivebase.Drivebase
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.launcher.Launcher
import frc.robot.subsystems.leds.LEDs
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [CommandRobot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to inputs reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object Robot {
//    val lowPowerMode = LoggedDashboardBoolean("Low Power Mode")

    init {
        registerFieldsForAutoLogOutput(
            DriverController,
            OperatorController,
            Drivebase,
            Launcher,
            LeftClimber,
            RightClimber,
            Intake,
            LEDs,
            Actions
        )

        configureBindings()
        makeDebugTab()
        makeButtonsTab()
    }

    private fun configureBindings() {
        DriverController.y().onTrue(Drivebase.resetGyroCommand())
//        DriverController.back().onTrue(Drivebase.toggleReverseModeCommand())
//        DriverController.start().onTrue(Drivebase.toggleArcadeDriveCommand())

//        DriverController.povUp().onTrue(SwerveMode.swerveAngleCommand(0.0))
//        DriverController.povLeft().onTrue(SwerveMode.swerveAngleCommand(90.0))
//        DriverController.povDown().onTrue(SwerveMode.swerveAngleCommand(180.0))
//        DriverController.povRight().onTrue(SwerveMode.swerveAngleCommand(-90.0))

//        DriverController.b().toggleOnTrue(Actions.doSelectedActionCommand())

        //        DriverController.b().toggleOnTrue(actions.doSelectedActionCommand());
//        DriverController.x().toggleOnTrue(actions.doSelectedActionWithoutAutoAlignCommand());
        OperatorController.leftTrigger(0.25).toggleOnTrue(Intake.handoffCommand().andThen(Launcher.launchCommand()))
        OperatorController.x().toggleOnTrue(Intake.ejectCommand())
        OperatorController.a().whileTrue(Launcher.intakeCommand())
        OperatorController.rightTrigger(0.25).whileTrue(Intake.intakeCommand())
//        DriverController.povUp().onTrue(Intake.resetPivotCommand())
//        DriverController.povDown().onTrue(Intake.pivotSlightlyDownCommand())
//        DriverController.x().onTrue(Drivebase.setPoseCommand(new Pose2d(1.41, 5.58, new Rotation2d()))); // subwoofer
//        DriverController.x().onTrue(Drivebase.setPoseCommand(new Pose2d(15.38, 0.958, Rotation2d.fromRadians(-0.9)))); // source
//        OperatorController.y().toggleOnTrue(Actions.selectActionCommand(Actions.Action.Source))
//        OperatorController.a().toggleOnTrue(Actions.selectActionCommand(Actions.Action.FrontSubwoofer))

        //        OperatorController.x().toggleOnTrue(actions.selectActionCommand(Actions.Action.LeftSubwoofer));
//        OperatorController.b().toggleOnTrue(actions.selectActionCommand(Actions.Action.RightSubwoofer));


//        DriverController.povUp()
//            .whileTrue(
//                RightClimber.moveCommand(Climber.Direction.Up)
////                LeftClimber.moveCommand(Climber.Direction.Up)
////                    .alongWith(RightClimber.moveCommand(Climber.Direction.Up))
//            )
//        DriverController.povDown()
//            .whileTrue(
//                RightClimber.moveCommand(Climber.Direction.Down)
////                LeftClimber.moveCommand(Climber.Direction.Down)
////                    .alongWith(RightClimber.moveCommand(Climber.Direction.Down))
//            )

//        DriverController.povUpLeft().whileTrue(RightClimber.moveCommand(Climber.Direction.Up))
//        DriverController.povUpRight().whileTrue(LeftClimber.moveCommand(Climber.Direction.Up))
//
//        DriverController.povDownLeft().whileTrue(RightClimber.moveCommand(Climber.Direction.Down))
//        DriverController.povDownRight().whileTrue(LeftClimber.moveCommand(Climber.Direction.Down))
    }

    private fun makeDebugTab() {
        val tab = Shuffleboard.getTab("Debug")
        tab.add("Command Scheduler", CommandScheduler.getInstance())
        tab.add("Drivebase", Drivebase)
        tab.add("Launcher", Launcher)
        tab.add("Left Climber", LeftClimber)
        tab.add("Right Climber", RightClimber)
        tab.add("Intake", Intake)
    }

    private fun makeButtonsTab() {
        val tab = Shuffleboard.getTab("Buttons")
        tab.add(
            "Zero pose to front of subwoofer",
            Drivebase.setPoseCommand(flipIfNeeded(Pose2d(1.33, 5.5, Rotation2d.fromDegrees(180.0))))
        )
        tab.add("Zero intake", Intake.resetPivotCommand())
        tab.add("Pivot intake down", Intake.pivotSlightlyDownCommand())
        tab.add("Reset right climber (from shooter side)", LeftClimber.resetCommand())
        tab.add("Reset left climber (from shooter side)", RightClimber.resetCommand())
    }

    private fun registerNamedCommands() {
        NamedCommands.registerCommand("Launch", Launcher.launchCommand())
        NamedCommands.registerCommand("Intake", Intake.intakeCommand())
        NamedCommands.registerCommand("Handoff", Intake.handoffCommand())
    }

    private val autoChooser = run {
        registerNamedCommands()
        val auto =
            LoggedDashboardChooser<Command?>("Auto")
        auto.addOption("None", null)
        // BROKEN
//        auto.addOption("Generate", Commands.deferredProxy(() -> AutoGenerator.generateAuto(Drivebase, launcher)));
        auto.addOption("Launch", Launcher.launchCommand())
        auto.addOption(
            "Intake and Launch",
            Intake.intakeCommand().andThen(Intake.handoffCommand(), Launcher.launchCommand())
        )
        auto.addDefaultOption("Launch and move", LaunchAndMove.get(Drivebase, Launcher))
//        auto.addOption("S2-W2-W1-W3", buildAllianceAuto("S2-W2-W1-W3"))
//        auto.addOption("S3-M5-M4", buildAllianceAuto("S3-M5-M4"))
        auto.addOption("S2-W2", buildAllianceAuto("S2-W2"))
        auto
    }

    fun getAutonomousCommand(): Command? {
        return autoChooser.get()
//        return null
    }

    fun teleopInit() {
        Drivebase.teleopInit()
    }
}