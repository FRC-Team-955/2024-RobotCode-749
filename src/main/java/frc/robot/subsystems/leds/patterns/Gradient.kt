package frc.robot.subsystems.leds.patterns

import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern
import kotlin.math.tanh

private const val tanhMultiplier = 6.0

/** totalDuration is the amount of time for one cycle */
class Gradient(
    val color1: Color,
    val color2: Color,
    val totalDuration: Double,
    val backwards: Boolean = false
) : LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        val offset = (timestamp % totalDuration / totalDuration * length).toInt()
        for ((i, led) in if (backwards) range.withIndex() else range.reversed().withIndex()) {
            var percent = (i + offset) % length / ((length - 1).toDouble() / 2)
            if (percent > 1) percent = 1 - (percent - 1)

            percent = (
                    tanh((percent - 0.5) * tanhMultiplier)
                            / tanh(0.5 * tanhMultiplier)
                    ) * 0.5 + 0.5

            val r = color2.red + (color1.red - color2.red) * percent
            val g = color2.green + (color1.green - color2.green) * percent
            val b = color2.blue + (color1.blue - color2.blue) * percent
            buffer.setLED(led, Color(r, g, b))
        }
    }

    fun backwards(): Gradient {
        return Gradient(color1, color2, totalDuration, true)
    }
}