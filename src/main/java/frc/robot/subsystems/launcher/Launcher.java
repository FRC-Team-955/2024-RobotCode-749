package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Util.switchMode;

public class Launcher extends SubsystemBase {
    private final LauncherIO io = switchMode(LauncherIOReal::new, LauncherIOSim::new, LauncherIO::new);
    private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Launcher", inputs);
    }

    public Command launchCommand() {
        return Commands.sequence(
                        this.runOnce(() -> io.setTopVoltage(LauncherConstants.launchingSpeed * 12)),
                        Commands.waitSeconds(1),
                        this.runOnce(() -> io.setBottomVoltage(LauncherConstants.launchingSpeed * 12)),
                        Commands.idle(this).withTimeout(1)
                )
                .finallyDo(io::stop);
    }

    public Command intakeCommand() {
        return this.startEnd(
                () -> {
                    io.setTopVoltage(LauncherConstants.topIntakeSpeed * 12);
                    io.setBottomVoltage(LauncherConstants.bottomIntakeSpeed * 12);
                },
                io::stop
        );
    }
}
