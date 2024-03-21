package frc.robot.subsystems.launcher

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.leds.LEDs
import frc.robot.switchMode
import org.littletonrobotics.junction.Logger

object Launcher : SubsystemBase() {
    private val io = switchMode(::LauncherIOReal, ::LauncherIOSim, ::LauncherIO)
    private val inputs = LauncherIO.inputs()

    private val spinUpTimer = Timer()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Inputs/Launcher", inputs)

        Logger.recordOutput("Launcher/SpinUpTimer", spinUpTimer.get())
    }

    fun launchCommand(): Command {
        return Commands.sequence(
            runOnce {
                io.setTopVoltage(Constants.Launcher.launchingSpeed * 12)
                spinUpTimer.stop()
            },
            // spinUpTimer.hasElapsed(Launcher.spinUpTime) ? Commands.none() : Commands.waitSeconds(Launcher.spinUpTime - spinUpTimer.get()),
            Commands.waitSeconds(Constants.Launcher.spinUpTime),
            runOnce {
                io.setBottomVoltage(Constants.Launcher.launchingSpeed * 12)
                spinUpTimer.reset()
            },
            Commands.idle(this).withTimeout(1.0)
        )
            .alongWith(LEDs.blinkCommand(Color.kAqua, Constants.LEDs.blinkDurationInProgress))
            .andThen(LEDs.blinkCommand(Color.kGreen, Constants.LEDs.blinkDurationCompleted))
            .finallyDo(Runnable { io.stop() })
            .withName("Launcher\$launch")
    }

    fun intakeCommand(): Command {
        return startEnd(
            {
                io.setTopVoltage(Constants.Launcher.topIntakeSpeed * 12)
                io.setBottomVoltage(Constants.Launcher.bottomIntakeSpeed * 12)
            },
            { io.stop() }
        ).withName("Launcher\$intake")
    }

    fun startSpinUpCommand(): Command {
        return runOnce {
//            io.setTopVoltage(Launcher.launchingSpeed * 12);
            spinUpTimer.restart()
        }.withName("Launcher\$startSpinUp")
    }

    fun stopSpinUpCommand(): Command {
        return runOnce {
            //            io.stop();
            spinUpTimer.stop()
            spinUpTimer.reset()
        }.withName("Launcher\$stopSpinUp")
    }
}
