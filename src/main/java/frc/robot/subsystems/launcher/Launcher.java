package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
                        runOnce(() -> {
                            io.setTopVoltage(Constants.Launcher.launchingSpeed * 12);
                            spinUpTimer.stop();
                        }),
//                        spinUpTimer.hasElapsed(Launcher.spinUpTime) ? Commands.none() : Commands.waitSeconds(Launcher.spinUpTime - spinUpTimer.get()),
                        Commands.waitSeconds(Constants.Launcher.spinUpTime),
                        runOnce(() -> {
                            io.setBottomVoltage(Constants.Launcher.launchingSpeed * 12);
                            spinUpTimer.reset();
                        }),
                        Commands.idle(this).withTimeout(1)
                )
                .finallyDo(io::stop)
                .withName("Launcher$launch");
    }

    public Command intakeCommand() {
        return startEnd(
                () -> {
                    io.setTopVoltage(Constants.Launcher.topIntakeSpeed * 12);
                    io.setBottomVoltage(Constants.Launcher.bottomIntakeSpeed * 12);
                },
                io::stop
        ).withName("Launcher$intake");
    }

    public Command startSpinUpCommand() {
        return runOnce(() -> {
//            io.setTopVoltage(Launcher.launchingSpeed * 12);
            spinUpTimer.restart();
        }).withName("Launcher$startSpinUp");
    }

    public Command stopSpinUpCommand() {
        return runOnce(() -> {
//            io.stop();
            spinUpTimer.stop();
            spinUpTimer.reset();
        }).withName("Launcher$stopSpinUp");
    }
}
