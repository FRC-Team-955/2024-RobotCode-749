package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;

import static frc.robot.util.IOUtil.chooseIO;

public class Launcher extends SubsystemBase {
    private final LauncherIO io = chooseIO(LauncherIOReal::new, LauncherIOSim::new, LauncherIO::new);

    private void startLaunching() {
        io.setTopVoltage(LauncherConstants.launchingSpeed * 12);
        io.setBottomVoltage(LauncherConstants.launchingSpeed * 12);
    }

    private void startIntake() {
        io.setTopVoltage(LauncherConstants.feederIntakeSpeed * 12);
        io.setBottomVoltage(LauncherConstants.launcherIntakeSpeed * 12);
    }

    private void stopMotors() {
        io.stop();
    }

    public Command launchCommand() {
        return this.startEnd(
                this::startLaunching,
                this::stopMotors
        );
    }

    public Command intakeCommand() {
        return this.startEnd(
                this::startIntake,
                this::stopMotors
        );
    }
}
