package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.controller.DriverController
import frc.robot.subsystems.controller.OperatorController
import frc.robot.subsystems.drivebase.Drivebase
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.launcher.Launcher
import org.littletonrobotics.junction.AutoLogOutput
import java.util.*

object Actions {
    @AutoLogOutput(key = "SelectedAction")
    private var selectedAction = Action.None

    @AutoLogOutput(key = "PrevSelectedAction")
    private var prevSelectedAction = Action.None

    private fun commandForAction(): Command {
        return when (selectedAction) {
            Action.Source -> Launcher.intakeCommand()
            Action.FrontSubwoofer, Action.LeftSubwoofer, Action.RightSubwoofer -> Intake.handoffCommand().andThen(Launcher.launchCommand())
            else -> Commands.none()
        }
    }

    private fun autoAlignForAction(boundsCheck: Boolean): Optional<Command> {
        return when (selectedAction) {
            Action.Source -> AutoAlign.sourceCommand(boundsCheck)
            Action.FrontSubwoofer -> AutoAlign.frontSubwooferCommand(boundsCheck)
            Action.LeftSubwoofer -> AutoAlign.leftSubwooferCommand(boundsCheck)
            Action.RightSubwoofer -> AutoAlign.rightSubwooferCommand(boundsCheck)
            else -> Optional.empty<Command>()
        }
    }

    private fun onSelectForAction(): Command {
        if (selectedAction == prevSelectedAction) return Commands.none()
        if (selectedAction == Action.FrontSubwoofer || selectedAction == Action.LeftSubwoofer || selectedAction == Action.RightSubwoofer
        ) return Launcher.startSpinUpCommand()
        if (selectedAction != Action.None) return Launcher.stopSpinUpCommand()
        return Commands.none()
    }

    private fun onReSelectForAction(): Command {
//        if (selectedAction != prevSelectedAction) return Commands.none();
//        if (selectedAction == Action.FrontSubwoofer ||
//                selectedAction == Action.LeftSubwoofer ||
//                selectedAction == Action.RightSubwoofer
//        ) return launcher.startSpinUpCommand();
        return Commands.none()
    }

    private fun checkForNone(): Optional<Command> {
        if (selectedAction == Action.None) return Optional.of<Command>(
            Commands.parallel(
                DriverController.setRumbleError(),
                OperatorController.setRumbleError()
            )
        )
        return Optional.empty()
    }

    fun selectActionCommand(action: Action): Command {
        return Commands.runOnce({
            prevSelectedAction = selectedAction
            selectedAction = action
        })
            .andThen(
                Commands.parallel(
                    Commands.deferredProxy { this.onSelectForAction() },
                    Commands.deferredProxy { this.onReSelectForAction() }
                ))
    }

    fun doSelectedActionCommand(): Command {
        return Commands.deferredProxy {
            checkForNone().orElse(
                autoAlignForAction(true)
                    .map<Command> { command: Command -> command.andThen(commandForAction()) as Command }
                    .orElse(DriverController.setRumbleError())
            )
        }
    }

    fun doSelectedActionWithoutBoundsCheckCommand(): Command {
        return Commands.deferredProxy {
            checkForNone().orElse(
                autoAlignForAction(false).orElse(Commands.print("No command even with bounds check disabled!"))
                    .andThen(commandForAction())
            )
        }
    }

    fun doSelectedActionWithoutAutoAlignCommand(): Command {
        return Commands.deferredProxy { checkForNone().orElse(commandForAction()) }
    }

    enum class Action {
        None,
        Source,
        FrontSubwoofer,
        LeftSubwoofer,
        RightSubwoofer
    }
}
