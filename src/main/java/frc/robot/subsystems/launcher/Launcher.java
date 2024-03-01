package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Util.switchMode;

public class Launcher extends SubsystemBase {
    private final LauncherIO io = switchMode(LauncherIOReal::new, LauncherIOSim::new, LauncherIO::new);
    private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

    private final Timer spinUpTimer = new Timer();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Launcher", inputs);

        Logger.recordOutput("Launcher/SpinUpTimer", spinUpTimer.get());
    }

    public Command launchCommand() {
        return Commands.sequence(
                        this.runOnce(() -> io.setTopVoltage(LauncherConstants.launchingSpeed * 12)),
                        spinUpTimer.get() < LauncherConstants.spinUpTime ? Commands.waitSeconds(LauncherConstants.spinUpTime - spinUpTimer.get()) : Commands.none(),
                        this.runOnce(() -> {
                            io.setBottomVoltage(LauncherConstants.launchingSpeed * 12);
                            spinUpTimer.stop();
                        }),
                        Commands.idle(this).withTimeout(1)
                )
                .finallyDo(io::stop)
                .withName("Launcher$launch");
    }

    public Command intakeCommand() {
        return this.startEnd(
                () -> {
                    io.setTopVoltage(LauncherConstants.topIntakeSpeed * 12);
                    io.setBottomVoltage(LauncherConstants.bottomIntakeSpeed * 12);
                },
                io::stop
        ).withName("Launcher$intake");
    }

    public Command startSpinUpCommand() {
        return this.runOnce(() -> {
            io.setTopVoltage(LauncherConstants.launchingSpeed * 12);
            spinUpTimer.restart();
        }).withName("Launcher$startSpinUpCommand");
    }

    public Command stopSpinUpCommand() {
        return this.runOnce(() -> {
            io.stop();
            spinUpTimer.stop();
        }).withName("Launcher$stopSpinUpCommand");
    }
}
