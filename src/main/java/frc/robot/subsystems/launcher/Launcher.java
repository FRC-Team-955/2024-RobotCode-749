package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;

import static frc.robot.Util.chooseIO;

public class Launcher extends SubsystemBase {
    private final LauncherIO io = chooseIO(LauncherIOReal::new, LauncherIOSim::new, LauncherIO::new);

    public Command launchCommand() {
        return Commands.sequence(
                        this.runOnce(() -> io.setTopVoltage(LauncherConstants.launchingSpeed * 12)),
                        Commands.waitSeconds(0.5),
                        this.runOnce(() -> io.setBottomVoltage(LauncherConstants.launchingSpeed * 12))
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
