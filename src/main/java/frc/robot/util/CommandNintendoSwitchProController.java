package frc.robot.util;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandNintendoSwitchProController extends CommandXboxController {
    public enum Button {
        LeftBumper(5),
        RightBumper(6),
        LeftStick(11),
        RightStick(12),
        // note: A is really 2, but having it as 1 matches the button position with an xbox controller
        A(1),
        // note: B is really 1, but having it as 2 matches the button position with an xbox controller
        B(2),
        // note: X is really 4, but having it as 1 matches the button position with an xbox controller
        X(3),
        // note: Y is really 3, but having it as 1 matches the button position with an xbox controller
        Y(4),
        Plus(10),
        Minus(9),
        LeftTrigger(7),
        RightTrigger(8);

        public final int value;

        Button(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            return this.name();
        }
    }

    public CommandNintendoSwitchProController(int port) {
        super(port);
    }

    @Override
    public Trigger leftBumper() {
        return button(Button.LeftBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightBumper() {
        return button(Button.RightBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger leftStick() {
        return button(Button.LeftStick.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightStick() {
        return button(Button.RightStick.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger a() {
        return button(Button.A.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger b() {
        return button(Button.B.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger x() {
        return button(Button.X.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger y() {
        return button(Button.Y.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger start() {
        return button(Button.Plus.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger back() {
        return button(Button.Minus.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger leftTrigger() {
        return button(Button.LeftTrigger.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightTrigger() {
        return button(Button.RightTrigger.value, CommandScheduler.getInstance().getDefaultButtonLoop());
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

    @Override
    public double getLeftTriggerAxis() {
        return getHID().getRawButton(Button.LeftTrigger.value) ? 1 : 0;
    }

    @Override
    public double getRightTriggerAxis() {
        return getHID().getRawButton(Button.RightTrigger.value) ? 1 : 0;
    }
}
