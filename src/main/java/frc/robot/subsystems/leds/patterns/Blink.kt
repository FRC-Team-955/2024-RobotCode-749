package frc.robot.subsystems.leds.patterns

import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.leds.BufferWrapper
import frc.robot.subsystems.leds.LEDPattern

/** Duration is how much time until swapping between on and off */
class Blink(val pattern: LEDPattern, val duration: Double) : LEDPattern() {
    override fun periodic(range: IntProgression, buffer: BufferWrapper, length: Int, timestamp: Double) {
        val on = timestamp % (duration * 2) < duration
        if (on)
            pattern.periodic(range, buffer, length, timestamp)
        else
            for (i in range)
                buffer.setLED(i, Color.kBlack)
    }
}