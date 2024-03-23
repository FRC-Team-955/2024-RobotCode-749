package frc.robot.subsystems.leds.patterns

import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern

class Split(val patterns: List<LEDPattern>) : LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        val patternSize = length / patterns.size
        var start = range.first
        for (pattern in patterns) {
            val end = start + patternSize
            pattern.periodic(start..<end, buffer, timestamp)
            start = end
        }
    }
}