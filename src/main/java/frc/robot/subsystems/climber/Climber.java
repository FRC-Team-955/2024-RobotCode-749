package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Util.switchMode;

public class Climber extends SubsystemBase {
    private final ClimberIO io = switchMode(ClimberIOReal::new, ClimberIOSim::new, ClimberIO::new);
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);
    }

    public Command setRightCommand(Direction direction) {
        return startEnd(() -> io.setRight(direction.speed), io::stop);
    }

    public Command setLeftCommand(Direction direction) {
        return startEnd(() -> io.setLeft(direction.speed), io::stop);
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
