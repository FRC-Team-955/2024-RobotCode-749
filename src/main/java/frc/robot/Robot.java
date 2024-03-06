package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoGenerator;
import frc.robot.auto.LaunchAndMove;
import frc.robot.commands.Actions;
import frc.robot.constants.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIORealLeft;
import frc.robot.subsystems.climber.ClimberIORealRight;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.util.CommandNintendoSwitchProController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Robot {
    private final CommandXboxController driverController = SimulationConstants.useNintendoSwitchProController ?
            new CommandNintendoSwitchProController(GeneralConstants.driverControllerPort) :
            new CommandXboxController(GeneralConstants.driverControllerPort);
    private final CommandXboxController operatorController = SimulationConstants.useNintendoSwitchProController ?
            new CommandNintendoSwitchProController(GeneralConstants.operatorControllerPort) :
            new CommandXboxController(GeneralConstants.operatorControllerPort);

    protected static final Class<?>[] constantClasses = new Class[]{
            ClimberConstants.class,
            DrivebaseConstants.class,
            GeneralConstants.class,
            LauncherConstants.class,
            SimulationConstants.class,
            IntakeConstants.class
    };

    /* Subsystems */
    private final Drivebase drivebase = new Drivebase(driverController);
    private final Launcher launcher = new Launcher();
    private final Climber leftClimber = new Climber(operatorController, "ClimberLeft", ClimberIORealLeft::new);
    private final Climber rightClimber = new Climber(operatorController, "ClimberRight", ClimberIORealRight::new);
    private final Intake intake = new Intake();

    private final Actions actions = new Actions(driverController, operatorController, drivebase, launcher);

    public Robot() {
        setDefaultCommands();
        configureBindings();
        makeDebugTab();
        makeButtonsTab();
    }

    private void setDefaultCommands() {
        drivebase.setDefaultCommand(drivebase.swerveMode.swerveDriveCommand());
    }

    private void configureBindings() {
        driverController.leftBumper().onTrue(drivebase.resetGyroCommand());
        driverController.start().onTrue(drivebase.toggleArcadeDrive());

        driverController.povUp().onTrue(drivebase.swerveMode.swerveAngleCommand(0));
        driverController.povLeft().onTrue(drivebase.swerveMode.swerveAngleCommand(90));
        driverController.povDown().onTrue(drivebase.swerveMode.swerveAngleCommand(180));
        driverController.povRight().onTrue(drivebase.swerveMode.swerveAngleCommand(-90));

        driverController.b().toggleOnTrue(actions.doSelectedActionWithoutAutoAlignCommand());
        driverController.a().toggleOnTrue(intake.moveDown());

//        driverController.b().toggleOnTrue(actions.doSelectedActionCommand());
//        driverController.x().toggleOnTrue(actions.doSelectedActionWithoutAutoAlignCommand());
//        driverController.a().toggleOnTrue(launcher.intakeCommand());
//        driverController.b().toggleOnTrue(launcher.launchCommand());
//        driverController.x().onTrue(drivebase.setPoseCommand(new Pose2d(1.41, 5.58, new Rotation2d()))); // subwoofer
//        driverController.x().onTrue(drivebase.setPoseCommand(new Pose2d(15.38, 0.958, Rotation2d.fromRadians(-0.9)))); // source

        operatorController.y().toggleOnTrue(actions.selectActionCommand(Actions.Action.Source));
        operatorController.a().toggleOnTrue(actions.selectActionCommand(Actions.Action.FrontSubwoofer));
        operatorController.x().toggleOnTrue(actions.selectActionCommand(Actions.Action.LeftSubwoofer));
        operatorController.b().toggleOnTrue(actions.selectActionCommand(Actions.Action.RightSubwoofer));

        // note: right and left are switched here to make it easier for the operator to control
        operatorController.rightBumper().whileTrue(leftClimber.moveCommand(Climber.Direction.Up));
        operatorController.rightTrigger().whileTrue(leftClimber.moveCommand(Climber.Direction.Down));
        operatorController.leftBumper().whileTrue(rightClimber.moveCommand(Climber.Direction.Up));
        operatorController.leftTrigger().whileTrue(rightClimber.moveCommand(Climber.Direction.Down));
        operatorController.povLeft().onTrue(rightClimber.resetCommand());
        operatorController.povRight().onTrue(leftClimber.resetCommand());
    }

    private void makeDebugTab() {
        var tab = Shuffleboard.getTab("Debug");
        tab.add("Command Scheduler", CommandScheduler.getInstance());
        tab.add("Drivebase", drivebase);
        tab.add("Launcher", launcher);
        tab.add("Left Climber", leftClimber);
        tab.add("Right Climber", rightClimber);
    }

    private void makeButtonsTab() {
        var tab = Shuffleboard.getTab("Buttons");
        tab.add("Zero pose to front of subwoofer", drivebase.setPoseCommand(Util.flipIfNeeded(new Pose2d(1.33, 5.5, Rotation2d.fromDegrees(180)))));
    }

    private final LoggedDashboardChooser<Command> autoChooser = Util.make(() -> {
        var auto = new LoggedDashboardChooser<Command>("Auto");
        auto.addDefaultOption("None", null);
        auto.addOption("Generate", AutoGenerator.generateAuto(drivebase, launcher));
        auto.addOption("Launch", launcher.launchCommand());
        auto.addOption("Launch and move", LaunchAndMove.get(drivebase, launcher));
        auto.addOption("Launch from front and move to corner", launcher.launchCommand()
                .andThen(
                        drivebase.followPathCommand("Front Subwoofer to Corner"),
                        Commands.parallel(
                                drivebase.swerveMode.swerveDriveCommand(),
                                Commands.deferredProxy(() -> drivebase.swerveMode.swerveAngleCommand(Util.shouldFlip() ? 180 : 0))
                        ).withTimeout(1.5)
                )
        );
        auto.addOption("Launch right and away", launcher.launchCommand().andThen(drivebase.followPathCommand("Right Away")));
        auto.addOption("Launch front and away", launcher.launchCommand().andThen(drivebase.followPathCommand("Front Away")));
        auto.addOption("Launch left and away", launcher.launchCommand().andThen(drivebase.followPathCommand("Left Away")));
        auto.addDefaultOption("Score Right and Corner", AutoBuilder.buildAuto("Score Right and Corner"));
        return auto;
    });

    public Command getAutonomousCommand() {
        // Return null to do nothing during autonomous.
        return autoChooser.get();
    }
}