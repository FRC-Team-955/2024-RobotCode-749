package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.launcher.Launcher;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class LaunchAndMove {
    private static final LoggedDashboardNumber speed = new LoggedDashboardNumber("Auto: Launch and Move: Speed", -0.5);
    private static final LoggedDashboardNumber rotation = new LoggedDashboardNumber("Auto: Launch and Move: Rotation", 0);
    private static final LoggedDashboardNumber seconds = new LoggedDashboardNumber("Auto: Launch and Move: Seconds", 1.5);

    public static Command get(Drivebase drivebase, Launcher launcher) {
        return Commands.deferredProxy(() ->
                launcher.launchCommand()
                        .andThen(drivebase.run(() -> drivebase.arcadeDrive(speed.get(), rotation.get()))
                                .withTimeout(seconds.get()))
        );
    }
}
