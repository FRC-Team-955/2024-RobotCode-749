package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Util.chooseIO;

public class Climber extends SubsystemBase {
    private final ClimberIO io = chooseIO(ClimberIOReal::new, ClimberIOSim::new, ClimberIO::new);
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);
    }

    /**
     * Sets the speed of the right motor and on interrupt, stops the motor.
     *
     * @param speed A positive value is up and a negative value is down.
     */
    public Command setRightCommand(double speed) {
        return startEnd(() -> io.setRight(speed), io::stop);
    }

    /**
     * Sets the speed of the left motor and on interrupt, stops the motor.
     *
     * @param speed A positive value is up and a negative value is down.
     */
    public Command setLeftCommand(double speed) {
        return startEnd(() -> io.setLeft(speed), io::stop);
    }
}
