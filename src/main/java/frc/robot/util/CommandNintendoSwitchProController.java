package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandNintendoSwitchProController extends CommandXboxController {
    public CommandNintendoSwitchProController(int port) {
        super(port);
    }

    @Override
    public double getLeftX() {
        return getRawAxis(0);
    }

    @Override
    public double getLeftY() {
        return -getRawAxis(1);
    }

    @Override
    public double getRightX() {
        return getRawAxis(2);
    }

    @Override
    public double getRightY() {
        return -getRawAxis(3);
    }

    // TODO: right and left triggers - need to override both Trigger and axis functions
    // TODO: fix + and - pressing sticks down
}
