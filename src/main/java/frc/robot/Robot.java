package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.Launcher;

public class Robot {
    private final CommandXboxController controller = new CommandXboxController(IOConstants.controllerPort);

    /* Subsystems */
    private final Launcher launcher = new Launcher();

    public Robot() {
        setDefaultCommands();
        configureBindings();
    }

    private void setDefaultCommands() {
    }

    private void configureBindings() {
        controller.x().onTrue(launcher.startLaunchingCommand().withTimeout(5).andThen(launcher.stopCommand()));
        controller.b().onTrue(launcher.startIntakeCommand().withTimeout(5).andThen(launcher.stopCommand()));
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing during autonomous.
        return null;
    }
}
