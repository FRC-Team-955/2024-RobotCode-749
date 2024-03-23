package frc.robot.subsystems.leds

import frc.robot.size

abstract class LEDPattern {
    fun periodic(range: IntProgression, buffer: BufferWrapper, timestamp: Double) {
        periodic(range, buffer, range.size(), timestamp)
    }

    abstract fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double)
}