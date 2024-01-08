package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.Drivebase;

public class Robot {
    private final CommandXboxController controller = new CommandXboxController(IOConstants.controllerPort);

    /* Subsystems */
    private final Drivebase drivebase = new Drivebase();

    public Robot() {
        setDefaultCommands();
        configureBindings();
    }

    private void setDefaultCommands() {
        drivebase.setDefaultCommand(drivebase.run(() -> drivebase.arcadeDrive(controller.getLeftY(), controller.getRightX())));
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing during autonomous.
        return null;
    }
}
