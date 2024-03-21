package frc.robot.subsystems.climber

import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs
import frc.robot.subsystems.controller.OperatorController
import frc.robot.subsystems.leds.LEDs
import frc.robot.switchMode
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier
import kotlin.math.abs

object LeftClimber : Climber("ClimberLeft", ::ClimberIORealLeft)
object RightClimber : Climber("ClimberRight", ::ClimberIORealRight)

open class Climber(
    private val name: String,
    realIO: Supplier<ClimberIO>
) : SubsystemBase() {
    private val io =
        switchMode(realIO, ::ClimberIOSim, ::ClimberIO)
    private val inputs: ClimberIOInputs = ClimberIO.inputs()

    private var ropeLeft = Constants.Climber.ropeLength

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Inputs/$name", inputs)

        ropeLeft = Constants.Climber.ropeLength - (abs(inputs.positionRad) / Constants.Climber.spoolDiameter)
        Logger.recordOutput("$name/RopeLeft", ropeLeft)
    }

    fun moveCommand(direction: Direction): Command {
        return MoveClimberCommand(direction)
    }

    fun resetCommand(): Command {
        return runOnce { io.resetPosition() }
    }

    enum class Direction(val speed: Double) {
        Up(1.0),
        Down(-1.0)
    }

    private inner class MoveClimberCommand(private val direction: Direction) : Command() {
        init {
            name = "Climber\$move"
        }

        override fun initialize() {
            if (direction == limitDirection() && ropeLeft <= Constants.Climber.ropeLeftThreshold) error()
            else io.set(direction.speed * 12.0)
        }

        override fun execute() {
            if (direction == limitDirection() && ropeLeft <= Constants.Climber.ropeLeftThreshold) error()
        }

        override fun end(interrupted: Boolean) {
            io.stop()
        }

        private fun limitDirection(): Direction {
            return if (inputs.positionRad > 0) Direction.Up else Direction.Down
        }

        private fun error() {
            OperatorController.setRumbleError().schedule()
            LEDs.blinkCommand(Color.kRed, Constants.LEDs.blinkDurationInProgress).withTimeout(1.0).schedule()
            cancel()
        }
    }
}
