package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.launcher.Launcher;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Optional;

public class Actions {
    private final Drivebase drivebase;
    private final Launcher launcher;

    @AutoLogOutput(key = "SelectedAction")
    private Action selectedAction = Action.None;

    public Actions(Drivebase drivebase, Launcher launcher) {
        this.drivebase = drivebase;
        this.launcher = launcher;
    }

    public Command selectActionCommand(Action action) {
        return Commands.runOnce(() -> selectedAction = action);
    }

    private Command commandForAction() {
        if (selectedAction == Action.Source) {
            return launcher.intakeCommand().withTimeout(7.5);
        } else if (selectedAction == Action.FrontSubwoofer ||
                selectedAction == Action.LeftSubwoofer ||
                selectedAction == Action.RightSubwoofer
        ) {
            return launcher.launchCommand();
        } else {
            return Commands.none();
        }
    }

    private Optional<Command> autoAlignForAction() {
        switch (selectedAction) {
            case Source -> {
                return drivebase.autoAlign.sourceCommand();
            }
            case FrontSubwoofer -> {
                return drivebase.autoAlign.frontSubwooferCommand();
            }
            case LeftSubwoofer -> {
                return drivebase.autoAlign.leftSubwooferCommand();
            }
            case RightSubwoofer -> {
                return drivebase.autoAlign.rightSubwooferCommand();
            }
            default -> {
                return Optional.empty();
            }
        }
    }

    public Command doSelectedActionCommand(CommandXboxController autoAlignController) {
        return Commands.deferredProxy(() ->
                autoAlignForAction()
                        .map(command -> (Command) command.andThen(commandForAction()))
                        .orElse(Controller.setRumbleError(autoAlignController))
        );
    }

    public Command doSelectedActionWithoutAutoAlignCommand() {
        return Commands.deferredProxy(this::commandForAction);
    }

    public enum Action {
        None,
        Source,
        FrontSubwoofer,
        LeftSubwoofer,
        RightSubwoofer
    }
}
