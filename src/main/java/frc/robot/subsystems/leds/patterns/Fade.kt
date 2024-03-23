package frc.robot.subsystems.leds.patterns

import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern
import kotlin.math.tanh

private const val tanhMultiplier = 3.0

/** totalDuration is the amount of time for one cycle */
class Fade(
    val color1: Color,
    val color2: Color,
    val totalDuration: Double,
) : LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        var percent = timestamp % totalDuration / (totalDuration / 2)
        if (percent > 1) percent = 1 - (percent - 1)

        percent = (
                tanh((percent - 0.5) * tanhMultiplier)
                        / tanh(0.5 * tanhMultiplier)
                ) * 0.5 + 0.5

        val r = color2.red + (color1.red - color2.red) * percent
        val g = color2.green + (color1.green - color2.green) * percent
        val b = color2.blue + (color1.blue - color2.blue) * percent

        val color = Color(r, g, b)
        for (i in range) {
            buffer.setLED(i, color)
        }
    }
}