package frc.robot.subsystems.leds.patterns

import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern

class Solid(val color: Color) : LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        for (i in range) {
            buffer.setLED(i, color)
        }
    }
}