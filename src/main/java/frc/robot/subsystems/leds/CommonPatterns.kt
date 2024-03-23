package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.leds.patterns.Gradient
import frc.robot.subsystems.leds.patterns.Rainbow
import frc.robot.subsystems.leds.patterns.Split

fun rainbowBothSides(totalDuration: Double, backwards: Boolean = false): LEDPattern {
    return Split(
        listOf(
            Rainbow(totalDuration, !backwards),
            Rainbow(totalDuration, backwards)
        )
    )
}

fun gradientBothSides(
    color1: Color,
    color2: Color,
    totalDuration: Double, backwards: Boolean = false
): LEDPattern {
    return Split(
        listOf(
            Gradient(color1, color2, totalDuration, !backwards),
            Gradient(color1, color2, totalDuration, backwards)
        )
    )
}