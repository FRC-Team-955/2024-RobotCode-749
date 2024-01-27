package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.util.CommandNintendoSwitchProController;

public class Robot {
    private final CommandXboxController controller = SimulationConstants.useNintendoSwitchProController ?
            new CommandNintendoSwitchProController(GeneralConstants.controllerPort) :
            new CommandXboxController(GeneralConstants.controllerPort);
    private final CommandXboxController controller2 = SimulationConstants.useNintendoSwitchProController ?
            new CommandNintendoSwitchProController(1) :
            new CommandXboxController(1);

    /* Subsystems */
    private final Drivebase drivebase = new Drivebase();
    private final Launcher launcher = new Launcher();

    public Robot() {
        setDefaultCommands();
        configureBindings();
    }

    private void setDefaultCommands() {
        drivebase.setDefaultCommand(drivebase.swerveDriveCommand(controller, false));
    }

    private void configureBindings() {
        controller.rightTrigger().whileTrue(drivebase.swerveDriveCommand(controller, true));
        controller.rightBumper().whileTrue(drivebase.arcadeDriveCommand(controller, false));
        controller.leftTrigger().whileTrue(drivebase.arcadeDriveCommand(controller, true));
        controller.leftBumper().onTrue(drivebase.resetPoseCommand());

        controller.povUp().onTrue(drivebase.swerveAngleCommand(0));
        controller.povLeft().onTrue(drivebase.swerveAngleCommand(90));
        controller.povDown().onTrue(drivebase.swerveAngleCommand(180));
        controller.povRight().onTrue(drivebase.swerveAngleCommand(270));

        controller.x().toggleOnTrue(launcher.launchCommand().withTimeout(5));
        controller.b().toggleOnTrue(launcher.intakeCommand().withTimeout(5));
        controller.a().toggleOnTrue(drivebase.followPathCommand("Subwoofer"));

        controller2.y().toggleOnTrue(drivebase.autoAlign.intakeSubwooferCommand());
        controller2.b().toggleOnTrue(drivebase.autoAlign.rightSubwooferCommand());
        controller2.x().toggleOnTrue(drivebase.autoAlign.leftSubwooferCommand());
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing during autonomous.
        return null;
    }
}
