package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.Util.switchMode;

public class Climber extends SubsystemBase {
    private final String name;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(String name, Supplier<ClimberIO> realIO) {
        this.name = name;
        this.io = switchMode(realIO, ClimberIOSim::new, ClimberIO::new);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + name, inputs);
    }

    public Command moveCommand(Direction direction) {
        return startEnd(() -> io.set(direction.speed * 12.0), io::stop);
    }

    public enum Direction {
        Up(1),
        Down(-1);

        private final double speed;

        Direction(double speed) {
            this.speed = speed;
        }
    }
}
