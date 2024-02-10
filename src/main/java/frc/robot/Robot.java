package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoGenerator;
import frc.robot.auto.Note;
import frc.robot.auto.StartingPoint;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.util.CommandNintendoSwitchProController;


public class Robot {
    private final CommandXboxController controller = SimulationConstants.useNintendoSwitchProController ?
            new CommandNintendoSwitchProController(GeneralConstants.controllerPort) :
            new CommandXboxController(GeneralConstants.controllerPort);

    /* Subsystems */
    private final Drivebase drivebase = new Drivebase();
    private final Launcher launcher = new Launcher();

    public Robot() {
        setDefaultCommands();
        configureBindings();
        AutoGenerator.initializeShuffleboard();
    }

    private void setDefaultCommands() {
        drivebase.setDefaultCommand(drivebase.swerveDriveCommand(controller, false));
    }

    private void configureBindings() {
        controller.rightTrigger().whileTrue(drivebase.swerveDriveCommand(controller, true));
        controller.rightBumper().whileTrue(drivebase.arcadeDriveCommand(controller, false));
        controller.leftTrigger().whileTrue(drivebase.arcadeDriveCommand(controller, true));
        controller.leftBumper().onTrue(drivebase.resetPoseCommand());

        controller.x().toggleOnTrue(launcher.launchCommand().withTimeout(5));
        controller.b().toggleOnTrue(launcher.intakeCommand().withTimeout(5));
        controller.a().toggleOnTrue(drivebase.followPathCommand("Subwoofer"));
    }

    public Command getAutonomousCommand() {

        // return AutoBuilder.buildAuto("Mess with all");
        // Return null to do nothing during autonomous.
        return AutoGenerator.generateAuto(drivebase);
    }
}
