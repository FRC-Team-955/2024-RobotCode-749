package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SimulationConstants;
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
    private final Drivebase drivebase = new Drivebase(driverController);
    private final Launcher launcher = new Launcher();

    public Robot() {
        setDefaultCommands();
        configureBindings();
    }

    private void setDefaultCommands() {
        drivebase.setDefaultCommand(drivebase.swerveDriveCommand(driverController, false));
    }

    private void configureBindings() {
        driverController.rightTrigger().whileTrue(drivebase.swerveDriveCommand(driverController, true));
        driverController.rightBumper().whileTrue(drivebase.arcadeDriveCommand(driverController, false));
        driverController.leftTrigger().whileTrue(drivebase.arcadeDriveCommand(driverController, true));
        driverController.leftBumper().onTrue(drivebase.resetPoseCommand());

        driverController.povUp().onTrue(drivebase.swerveAngleCommand(0));
        driverController.povLeft().onTrue(drivebase.swerveAngleCommand(90));
        driverController.povDown().onTrue(drivebase.swerveAngleCommand(180));
        driverController.povRight().onTrue(drivebase.swerveAngleCommand(270));

        driverController.x().toggleOnTrue(launcher.launchCommand().withTimeout(5));
        driverController.b().toggleOnTrue(launcher.intakeCommand().withTimeout(5));
        driverController.a().toggleOnTrue(drivebase.followPathCommand("Subwoofer"));

        operatorController.y().toggleOnTrue(drivebase.autoAlign.intakeSubwooferCommand());
        operatorController.b().toggleOnTrue(drivebase.autoAlign.rightSubwooferCommand());
        operatorController.x().toggleOnTrue(drivebase.autoAlign.leftSubwooferCommand());
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing during autonomous.
        return null;
    }
}
