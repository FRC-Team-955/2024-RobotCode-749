package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SimulationConstants;
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

    /* Subsystems */
    private final Drivebase drivebase = new Drivebase();
    private final Launcher launcher = new Launcher();
    private final Climber climber = new Climber();

    public Robot() {
        setDefaultCommands();
        configureBindings();
    }

    private void setDefaultCommands() {
        drivebase.setDefaultCommand(drivebase.swerveMode.swerveDriveCommand(driverController, false));
    }

    private void configureBindings() {
        driverController.rightTrigger().whileTrue(drivebase.swerveMode.swerveDriveCommand(driverController, true));
        driverController.rightBumper().whileTrue(drivebase.arcadeDriveCommand(driverController, false));
        driverController.leftTrigger().whileTrue(drivebase.arcadeDriveCommand(driverController, true));
        driverController.leftBumper().onTrue(drivebase.resetPoseCommand());

        driverController.povUp().onTrue(drivebase.swerveMode.swerveAngleCommand(0));
        driverController.povLeft().onTrue(drivebase.swerveMode.swerveAngleCommand(90));
        driverController.povDown().onTrue(drivebase.swerveMode.swerveAngleCommand(180));
        driverController.povRight().onTrue(drivebase.swerveMode.swerveAngleCommand(270));

        driverController.x().toggleOnTrue(launcher.launchCommand().withTimeout(5));
        driverController.b().toggleOnTrue(launcher.intakeCommand().withTimeout(5));

        operatorController.b().toggleOnTrue(drivebase.autoAlign.rightSubwooferCommand(driverController));
        operatorController.x().toggleOnTrue(drivebase.autoAlign.leftSubwooferCommand(driverController));
        operatorController.y().toggleOnTrue(drivebase.autoAlign.sourceCommand(driverController));

        operatorController.povUp().whileTrue(climber.setRightCommand(1));
        operatorController.povDown().whileTrue(climber.setRightCommand(-1));
        operatorController.povLeft().whileTrue(climber.setLeftCommand(1));
        operatorController.povRight().whileTrue(climber.setLeftCommand(-1));

        // This was for my keyboard.
        // operatorController.y().whileTrue(climber.setRightCommand(1));
        // operatorController.a().whileTrue(climber.setRightCommand(-1));
        // operatorController.x().whileTrue(climber.setLeftCommand(1));
        // operatorController.b().whileTrue(climber.setLeftCommand(-1));
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing during autonomous.
        return null;
    }
}
