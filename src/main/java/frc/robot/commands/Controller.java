package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.GeneralConstants;

public class Controller {
    public static Command setRumble(CommandXboxController controller, double rumble, double seconds) {
        return Commands.sequence(
                Commands.print("Activating rumble"),
                Commands.runOnce(() -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumble)),
                Commands.waitSeconds(seconds),
                Commands.runOnce(() -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0))
        );
    }

    public static Command setRumbleError(CommandXboxController controller) {
        return setRumble(controller, GeneralConstants.errorRumbleAmount, GeneralConstants.errorRumbleDuration);
    }
}
