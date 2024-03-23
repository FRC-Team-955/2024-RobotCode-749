package frc.robot.subsystems.leds.patterns

import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern

class Switching(val patterns: List<LEDPattern>, var durationPer: Double) : LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        val currentPattern = timestamp % (durationPer * patterns.size) / durationPer
        patterns.getOrNull(currentPattern.toInt())?.periodic(range, buffer, timestamp)
    }
}