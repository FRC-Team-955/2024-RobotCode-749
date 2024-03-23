package frc.robot.subsystems.leds.patterns

import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern
import kotlin.math.PI
import kotlin.math.sin

private const val lengthMultiplier = 8
private const val sinMultiplier = 2.0 / lengthMultiplier

/** totalDuration is the amount of time for one cycle */
class Wave(
    val bgColor: Color,
    val waveColor: Color,
    val totalDuration: Double,
    val backwards: Boolean = false
) : LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        val offset = (timestamp % totalDuration / totalDuration * lengthMultiplier * length).toInt()
        for ((i, led) in if (backwards) range.withIndex() else range.reversed().withIndex()) {
            var percent = (i + offset) % (lengthMultiplier * length) / length.toDouble()

            percent = sin(sinMultiplier * PI * percent)

            val r = bgColor.red + (waveColor.red - bgColor.red) * percent
            val g = bgColor.green + (waveColor.green - bgColor.green) * percent
            val b = bgColor.blue + (waveColor.blue - bgColor.blue) * percent
            buffer.setLED(led, Color(r, g, b))
        }
    }

    fun backwards(): Wave {
        return Wave(bgColor, waveColor, totalDuration, true)
    }
}