package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.launcher.Launcher;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Optional;

public class Actions {
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    private final Drivebase drivebase;
    private final Launcher launcher;

    @AutoLogOutput(key = "SelectedAction")
    private Action selectedAction = Action.None;

    @AutoLogOutput(key = "PrevSelectedAction")
    private Action prevSelectedAction = Action.None;

    public Actions(CommandXboxController driverController, CommandXboxController operatorController, Drivebase drivebase, Launcher launcher) {
        this.driverController = driverController;
        this.operatorController = operatorController;

        this.drivebase = drivebase;
        this.launcher = launcher;
    }

    private Command commandForAction() {
        if (selectedAction == Action.Source) {
            return launcher.intakeCommand();
        } else if (selectedAction == Action.FrontSubwoofer ||
                selectedAction == Action.LeftSubwoofer ||
                selectedAction == Action.RightSubwoofer
        ) {
            return launcher.launchCommand();
        } else {
            return Commands.none();
        }
    }

    private Optional<Command> autoAlignForAction(boolean boundsCheck) {
        switch (selectedAction) {
            case Source -> {
                return drivebase.autoAlign.sourceCommand(boundsCheck);
            }
            case FrontSubwoofer -> {
                return drivebase.autoAlign.frontSubwooferCommand(boundsCheck);
            }
            case LeftSubwoofer -> {
                return drivebase.autoAlign.leftSubwooferCommand(boundsCheck);
            }
            case RightSubwoofer -> {
                return drivebase.autoAlign.rightSubwooferCommand(boundsCheck);
            }
            default -> {
                return Optional.empty();
            }
        }
    }

    private Command onSelectForAction() {
        if (selectedAction == prevSelectedAction) return Commands.none();
        if (selectedAction != Action.None) return launcher.stopSpinUpCommand();
        return Commands.none();
    }

    private Command onReSelectForAction() {
        if (selectedAction != prevSelectedAction) return Commands.none();
        if (selectedAction == Action.Source) return launcher.stopSpinUpCommand();
        if (selectedAction == Action.FrontSubwoofer ||
                selectedAction == Action.LeftSubwoofer ||
                selectedAction == Action.RightSubwoofer
        ) return launcher.startSpinUpCommand();
        return Commands.none();
    }

    private Optional<Command> checkForNone() {
        if (selectedAction == Action.None) return Optional.of(Commands.parallel(
                Controller.setRumbleError(driverController),
                Controller.setRumbleError(operatorController)
        ));
        return Optional.empty();
    }

    public Command selectActionCommand(Action action) {
        return Commands.runOnce(() -> {
                    prevSelectedAction = selectedAction;
                    selectedAction = action;
                })
                .andThen(Commands.parallel(
                        Commands.deferredProxy(this::onSelectForAction),
                        Commands.deferredProxy(this::onReSelectForAction)
                ));
    }

    public Command doSelectedActionCommand() {
        return Commands.deferredProxy(() ->
                checkForNone().orElse(
                        autoAlignForAction(true)
                                .map(command -> (Command) command.andThen(commandForAction()))
                                .orElse(Controller.setRumbleError(driverController))
                )
        );
    }

    public Command doSelectedActionWithoutBoundsCheckCommand() {
        return Commands.deferredProxy(() ->
                checkForNone().orElse(
                        autoAlignForAction(false).orElse(Commands.print("No command even with bounds check disabled!"))
                                .andThen(commandForAction())
                )
        );
    }

    public Command doSelectedActionWithoutAutoAlignCommand() {
        return Commands.deferredProxy(() -> checkForNone().orElse(commandForAction()));
    }

    public enum Action {
        None,
        Source,
        FrontSubwoofer,
        LeftSubwoofer,
        RightSubwoofer
    }
}
