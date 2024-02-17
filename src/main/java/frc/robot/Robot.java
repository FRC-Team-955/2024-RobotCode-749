package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoGenerator;
import frc.robot.commands.Actions;
import frc.robot.constants.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.util.CommandNintendoSwitchProController;

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
            LimelightConstants.class,
            SimulationConstants.class
    };

    /* Subsystems */
    private final Drivebase drivebase = new Drivebase();
    private final Launcher launcher = new Launcher();
    private final Climber climber = new Climber();

    private final Actions actions = new Actions(drivebase, launcher);

    public Robot() {
        setDefaultCommands();
        configureBindings();
        AutoGenerator.initializeShuffleboard();
    }

    private void setDefaultCommands() {
        drivebase.setDefaultCommand(drivebase.swerveMode.swerveDriveCommand(driverController));
    }

    private void configureBindings() {
        driverController.rightBumper().onTrue(drivebase.toggleArcadeDrive(driverController));
        driverController.rightTrigger()
                .onTrue(drivebase.setReverseModeCommand(true))
                .onFalse(drivebase.setReverseModeCommand(false));
        driverController.leftTrigger()
                .onTrue(drivebase.setPreciseModeCommand(true))
                .onFalse(drivebase.setPreciseModeCommand(false));

        driverController.povUp().onTrue(drivebase.swerveMode.swerveAngleCommand(0));
        driverController.povLeft().onTrue(drivebase.swerveMode.swerveAngleCommand(90));
        driverController.povDown().onTrue(drivebase.swerveMode.swerveAngleCommand(180));
        driverController.povRight().onTrue(drivebase.swerveMode.swerveAngleCommand(270));

        driverController.b().toggleOnTrue(actions.doSelectedActionCommand(driverController));
        driverController.x().toggleOnTrue(actions.doSelectedActionWithoutAutoAlignCommand());

        operatorController.y().toggleOnTrue(actions.selectActionCommand(Actions.Action.Source));
        operatorController.a().toggleOnTrue(actions.selectActionCommand(Actions.Action.FrontSubwoofer));
        operatorController.x().toggleOnTrue(actions.selectActionCommand(Actions.Action.LeftSubwoofer));
        operatorController.b().toggleOnTrue(actions.selectActionCommand(Actions.Action.RightSubwoofer));

        operatorController.rightBumper().whileTrue(climber.setRightCommand(Climber.Direction.Up));
        operatorController.rightTrigger().whileTrue(climber.setRightCommand(Climber.Direction.Down));
        operatorController.leftBumper().whileTrue(climber.setLeftCommand(Climber.Direction.Up));
        operatorController.leftTrigger().whileTrue(climber.setLeftCommand(Climber.Direction.Down));
    }

    public Command getAutonomousCommand() {

        // return AutoBuilder.buildAuto("Mess with all");
        // Return null to do nothing during autonomous.
        return AutoGenerator.generateAuto(drivebase, launcher);
    }
}
