package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.Controller;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.Util.switchMode;

public class Climber extends SubsystemBase {
    private final CommandXboxController operatorController;
    private final String name;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private double ropeLeft = Constants.Climber.ropeLength;

    public Climber(CommandXboxController operatorController, String name, Supplier<ClimberIO> realIO) {
        this.operatorController = operatorController;
        this.name = name;
        this.io = switchMode(realIO, ClimberIOSim::new, ClimberIO::new);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + name, inputs);

        ropeLeft = Constants.Climber.ropeLength - (Math.abs(inputs.positionRad) / Constants.Climber.spoolDiameter);
        Logger.recordOutput(name + "/RopeLeft", ropeLeft);
    }

    public Command moveCommand(Direction direction) {
        return new MoveClimberCommand(direction);
    }

    public Command resetCommand() {
        return runOnce(io::resetPosition);
    }

    public enum Direction {
        Up(1),
        Down(-1);

        private final double speed;

        Direction(double speed) {
            this.speed = speed;
        }
    }

    private class MoveClimberCommand extends Command {
        private final Direction direction;

        private MoveClimberCommand(Direction direction) {
            this.direction = direction;
            setName("Climber$move");
        }

        @Override
        public void initialize() {
            if (direction == limitDirection() && ropeLeft <= Constants.Climber.ropeLeftThreshold)
                error();
            else
                io.set(direction.speed * 12.0);
        }

        @Override
        public void execute() {
            if (direction == limitDirection() && ropeLeft <= Constants.Climber.ropeLeftThreshold)
                error();
        }

        @Override
        public void end(boolean interrupted) {
            io.stop();
        }

        private Direction limitDirection() {
            return inputs.positionRad > 0 ? Direction.Up : Direction.Down;
        }

        private void error() {
            Controller.setRumbleError(operatorController).schedule();
            cancel();
        }
    }
}
