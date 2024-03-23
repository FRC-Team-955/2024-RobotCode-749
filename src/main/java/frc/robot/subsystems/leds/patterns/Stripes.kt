package frc.robot.subsystems.leds.patterns

import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern

/** durationPerStripe is the amount of time for one pattern to move to the next pattern's position */
class Stripes(val patterns: List<LEDPattern>, var durationPerStripe: Double, val backwards: Boolean = false) :
    LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        val patternSize = length / patterns.size
        println(patternSize)
        val offset = (timestamp / durationPerStripe * patternSize).toInt()
        var start = (range.first + offset) % length
        for (pattern in if (backwards) patterns.reversed() else patterns) {
            val end = start + patternSize
            pattern.periodic(start..<end, buffer, timestamp)
            start = end
        }
    }
}