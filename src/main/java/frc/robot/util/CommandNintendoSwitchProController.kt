package frc.robot.util

import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

class CommandNintendoSwitchProController(port: Int) : CommandXboxController(port) {
    enum class Button(val value: Int) {
        LeftBumper(5),
        RightBumper(6),
        LeftStick(11),
        RightStick(12),
        A(1), // note: A is really 2, but having it as 1 matches the button position with an xbox controller
        B(2), // note: B is really 1, but having it as 2 matches the button position with an xbox controller
        X(3), // note: X is really 4, but having it as 3 matches the button position with an xbox controller
        Y(4), // note: Y is really 3, but having it as 4 matches the button position with an xbox controller
        Plus(10),
        Minus(9),
        LeftTrigger(7),
        RightTrigger(8);

        override fun toString(): String {
            return this.name
        }
    }

    override fun leftBumper(): Trigger {
        return button(Button.LeftBumper.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun rightBumper(): Trigger {
        return button(Button.RightBumper.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun leftStick(): Trigger {
        return button(Button.LeftStick.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun rightStick(): Trigger {
        return button(Button.RightStick.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun a(): Trigger {
        return button(Button.A.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun b(): Trigger {
        return button(Button.B.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun x(): Trigger {
        return button(Button.X.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun y(): Trigger {
        return button(Button.Y.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun start(): Trigger {
        return button(Button.Plus.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun back(): Trigger {
        return button(Button.Minus.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun leftTrigger(): Trigger {
        return button(Button.LeftTrigger.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun rightTrigger(): Trigger {
        return button(Button.RightTrigger.value, CommandScheduler.getInstance().defaultButtonLoop)
    }

    override fun getLeftX(): Double {
        return getRawAxis(0)
    }

    override fun getLeftY(): Double {
        return getRawAxis(1)
    }

    override fun getRightX(): Double {
        return getRawAxis(2)
    }

    override fun getRightY(): Double {
        return getRawAxis(3)
    }

    override fun getLeftTriggerAxis(): Double {
        return if (hid.getRawButton(Button.LeftTrigger.value)) 1.0 else 0.0
    }

    override fun getRightTriggerAxis(): Double {
        return if (hid.getRawButton(Button.RightTrigger.value)) 1.0 else 0.0
    }
}
