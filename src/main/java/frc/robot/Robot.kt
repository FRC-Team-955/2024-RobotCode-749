package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.climber.LeftClimber
import frc.robot.subsystems.climber.RightClimber
import frc.robot.subsystems.controller.DriverController
import frc.robot.subsystems.controller.OperatorController
import frc.robot.subsystems.drivebase.Drivebase
import frc.robot.subsystems.drivebase.commands.SwerveMode
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.launcher.Launcher
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
    init {
        initializeSubsystems()
        configureBindings()
        makeDebugTab()
        makeButtonsTab()
    }

    private fun initializeSubsystems() {
        registerFieldsForAutoLogOutput(
            DriverController,
            OperatorController,
            Drivebase,
            Launcher,
            LeftClimber,
            RightClimber,
            Intake
        )
    }

    private fun configureBindings() {
        DriverController.leftBumper().onTrue(Drivebase.resetGyroCommand())
        DriverController.rightBumper().onTrue(Drivebase.toggleReverseModeCommand())
        DriverController.start().onTrue(Drivebase.toggleArcadeDrive())

        DriverController.povUp().onTrue(SwerveMode.swerveAngleCommand(0.0))
        DriverController.povLeft().onTrue(SwerveMode.swerveAngleCommand(90.0))
        DriverController.povDown().onTrue(SwerveMode.swerveAngleCommand(180.0))
        DriverController.povRight().onTrue(SwerveMode.swerveAngleCommand(-90.0))

//        DriverController.b().toggleOnTrue(actions.doSelectedActionWithoutAutoAlignCommand())

        //        DriverController.b().toggleOnTrue(actions.doSelectedActionCommand());
//        DriverController.x().toggleOnTrue(actions.doSelectedActionWithoutAutoAlignCommand());
//        DriverController.b().toggleOnTrue(launcher.launchCommand());
//        DriverController.a().toggleOnTrue(intake.handoffCommand());
//        DriverController.x().onTrue(Drivebase.setPoseCommand(new Pose2d(1.41, 5.58, new Rotation2d()))); // subwoofer
//        DriverController.x().onTrue(Drivebase.setPoseCommand(new Pose2d(15.38, 0.958, Rotation2d.fromRadians(-0.9)))); // source
//        OperatorController.y().toggleOnTrue(actions.selectActionCommand(Actions.Action.Source))
//        OperatorController.a().toggleOnTrue(actions.selectActionCommand(Actions.Action.FrontSubwoofer))

        //        OperatorController.x().toggleOnTrue(actions.selectActionCommand(Actions.Action.LeftSubwoofer));
//        OperatorController.b().toggleOnTrue(actions.selectActionCommand(Actions.Action.RightSubwoofer));
        OperatorController.b().toggleOnTrue(Intake.ejectCommand())
        Trigger { OperatorController.leftY < -0.6 }
            .onTrue(Intake.intakeCommand())
            .onFalse(Intake.tuckCommand())
        Trigger { OperatorController.leftY > 0.6 }
            .onTrue(Intake.pivotSlightlyDownCommand())
            .onFalse(Intake.tuckCommand())
        OperatorController.povUp().onTrue(Intake.resetPivotCommand())

        // note: right and left are switched here to make it easier for the operator to control
        OperatorController.rightBumper()
            .whileTrue(LeftClimber.moveCommand(Climber.Direction.Up))
        OperatorController.rightTrigger()
            .whileTrue(LeftClimber.moveCommand(Climber.Direction.Down))
        OperatorController.leftBumper()
            .whileTrue(RightClimber.moveCommand(Climber.Direction.Up))
        OperatorController.leftTrigger()
            .whileTrue(RightClimber.moveCommand(Climber.Direction.Down))
        OperatorController.povLeft().onTrue(RightClimber.resetCommand())
        OperatorController.povRight().onTrue(LeftClimber.resetCommand())
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
//        auto.addDefaultOption("Launch and move", LaunchAndMove.get(Drivebase, launcher))
//        auto.addOption("S2-W2-W1-W3", buildAllianceAuto("S2-W2-W1-W3"))
//        auto.addOption("S3-M5-M4", buildAllianceAuto("S3-M5-M4"))
        auto
    }

    fun getAutonomousCommand(): Command? {
        return autoChooser.get()
    }

    fun teleopInit() {
        Drivebase.teleopInit()
    }
}