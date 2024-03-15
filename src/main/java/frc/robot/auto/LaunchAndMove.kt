package frc.robot.auto

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.drivebase.Drivebase
import frc.robot.subsystems.launcher.Launcher
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber

object LaunchAndMove {
    private val speed = LoggedDashboardNumber("Auto: Launch and Move: Speed", -0.5)
    private val rotation = LoggedDashboardNumber("Auto: Launch and Move: Rotation", 0.0)
    private val seconds = LoggedDashboardNumber("Auto: Launch and Move: Seconds", 1.5)

    fun get(drivebase: Drivebase, launcher: Launcher): Command {
        return Commands.deferredProxy {
            Launcher.launchCommand()
                .andThen(drivebase.run { drivebase.arcadeDrive(speed.get(), rotation.get()) }
                    .withTimeout(seconds.get()))
        }
    }
}
