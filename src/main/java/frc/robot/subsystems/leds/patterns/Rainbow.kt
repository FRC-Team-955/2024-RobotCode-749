package frc.robot.subsystems.leds.patterns

import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern

/** totalDuration is the amount of time for one cycle */
class Rainbow(val totalDuration: Double, val backwards: Boolean = false) : LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        val offset = (timestamp % totalDuration / totalDuration * length).toInt()
        for ((i, led) in if (backwards) range.withIndex() else range.reversed().withIndex()) {
            val percent = (i + offset) % length / length.toDouble()
            buffer.setHSV(led, (180 * percent).toInt(), 255, 255)
        }
        // Alternate implementation, backwards does nothing
//        for (i in if (backwards) range.reversed() else range) {
//            val percent = i / length.toDouble()
//            buffer.setHSV(range.first + (i + offset) % length, (180 * percent).toInt(), 255, 255)
//        }
    }

    fun backwards(): Rainbow {
        return Rainbow(totalDuration, true)
    }
}