package frc.robot.subsystems.controller

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.util.CommandNintendoSwitchProController

object DriverController : Controller(
    if (Constants.Simulation.useNintendoSwitchProController) CommandNintendoSwitchProController(Constants.driverControllerPort)
    else CommandXboxController(Constants.driverControllerPort)
)

object OperatorController : Controller(
    if (Constants.Simulation.useNintendoSwitchProController) CommandNintendoSwitchProController(Constants.operatorControllerPort)
    else CommandXboxController(Constants.operatorControllerPort)
)

abstract class Controller(private val controller: CommandXboxController) : SubsystemBase() {
    fun setRumble(rumble: Double, seconds: Double): Command {
        return startEnd(
            {
                println("Activating rumble")
                hid.setRumble(GenericHID.RumbleType.kBothRumble, rumble)
            },
            {
                hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            }
        ).withTimeout(seconds)
    }

    fun setRumbleError(): Command {
        return setRumble(Constants.errorRumbleAmount, Constants.errorRumbleDuration)
    }

    fun speed(): Double {
        return -leftTriggerAxis + rightTriggerAxis
    }

    // CommandXboxController methods //

    val hid: XboxController
        get() = controller.hid

    fun leftBumper(): Trigger = controller.leftBumper()

    fun rightBumper(): Trigger = controller.rightBumper()

    fun leftStick(): Trigger = controller.leftStick()

    fun rightStick(): Trigger = controller.rightStick()

    fun a(): Trigger = controller.a()

    fun b(): Trigger = controller.b()

    fun x(): Trigger = controller.x()

    fun y(): Trigger = controller.y()

    fun start(): Trigger = controller.start()

    fun back(): Trigger = controller.back()

    fun povUp(): Trigger = controller.povUp()

    fun povDown(): Trigger = controller.povDown()

    fun povLeft(): Trigger = controller.povLeft()

    fun povRight(): Trigger = controller.povRight()

    fun leftTrigger(): Trigger = controller.leftTrigger()

    fun rightTrigger(): Trigger = controller.rightTrigger()

    val leftX: Double
        get() = controller.leftX

    val leftY: Double
        get() = controller.leftY

    val rightX: Double
        get() = controller.rightX

    val rightY: Double
        get() = controller.rightY

    val leftTriggerAxis: Double
        get() = controller.leftTriggerAxis

    val rightTriggerAxis: Double
        get() = controller.rightTriggerAxis
}