package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.Constants.LEDs.length
import frc.robot.subsystems.drivebase.Drivebase
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean

object LEDs : SubsystemBase() {
    private val buffer = AddressableLEDBuffer(length)
    private val led = kotlin.run {
        val led = AddressableLED(Constants.LEDs.id)

        // WS2811
//        led.setSyncTime(300)
//        led.setBitTiming(500, 2000, 1200, 1300)

        led.setLength(length)
        led.setData(buffer)
        led.start()
        led
    }

    init {
        defaultCommand = run {
            if (DriverStation.isEStopped()) estopped()
            else if (DriverStation.isAutonomousEnabled()) autonomous()
            else if (DriverStation.isTeleopEnabled()) teleop()
            else disabled()
        }.ignoringDisable(true).withName("LEDs\$passive")

        Trigger { RobotController.getBatteryVoltage() <= Constants.LEDs.lowBatteryVoltsThreshold }
            .onTrue(Commands.runOnce({ lowBattery = true }))
            .onFalse(Commands.runOnce({ lowBattery = false }))
        Trigger { DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() < Constants.LEDs.endgameThreshold }
            .onTrue(Commands.startEnd({ endgameAlert = true }, { endgameAlert = false }).withTimeout(5.0))
        // autoFinished is set in CommandRobot#autonomousPeriodic for simplicity
    }

    override fun periodic() {
        if (Constants.LEDs.debugPrint) {
            for (i in 0..<length)
                print("\u001b[38;2;${buffer.getRed(i)};${buffer.getGreen(i)};${buffer.getBlue(i)}m■\u001b[0m")
            println()
        }

        led.setData(buffer)
    }

    var autoFinished = false
    var lowBattery = false
    var endgameAlert = false
    val prideWhileDisabled = LoggedDashboardBoolean("Pride LEDs while disabled", false)

    private val lowBatteryColor = Color.kRed
    private val lowBatteryBlinkDuration = 0.5
    private val lowBatteryStripe = Stripe(lowBatteryColor, lowBatteryBlinkDuration)

    private fun estopped() {
        solid(Color.kRed)
    }

    private fun disabled() {
        if (lowBattery) {
            blink(lowBatteryColor, lowBatteryBlinkDuration)
            return
        }

        if (prideWhileDisabled.get()) {
            stripes(
                0.2,
                Stripe(Color.kWhite),
                Stripe(Color.kDeepPink),
                Stripe(Color.kCyan),
                Stripe(Color(162, 85, 42)),
                Stripe(Color.kBlack),
                Stripe(Color.kRed),
                Stripe(Color.kOrange),
                Stripe(Color.kYellow),
                Stripe(Color.kGreen),
                Stripe(Color.kBlue),
                Stripe(Color.kPurple),
            )
            return
        }

        val switchDuration = 10.0
        switchBetween(
            switchDuration,
            { solid(Color.kDeepPink) },
            { rainbow(switchDuration / 4) }
        )
    }

    private fun autonomous() {
        var color = if (autoFinished) Color.kGreen else Color.kYellow
        var blinkDuration = if (autoFinished) null else 0.5
        if (lowBattery) {
            var stripe = Stripe(color, blinkDuration)
            stripes(
                stripe,
                stripe,
                stripe,
                lowBatteryStripe
            )
        } else if (blinkDuration != null) blink(color, blinkDuration)
        else solid(color)
    }

    private fun teleop() {
        var reverseColor = if (Drivebase.reverseMode) Color.kRed else Color.kGreen
        if (endgameAlert)
            stripes(Stripe(reverseColor), Stripe(Color.kYellow, 0.5))
        else solid(reverseColor)
    }

    // LED control functions //

    private fun solid(color: Color) {
        for (i in 0..<length) {
            buffer.setLED(i, color)
        }
    }

    private fun blink(color: Color, duration: Double) {
        blink(color, Color.kBlack, duration)
    }

    fun blinkCommand(color: Color, duration: Double): Command {
        return run {
            blink(color, duration)
        }.withName("LEDs\$blink")
    }

    private fun blink(color1: Color, color2: Color, duration: Double) {
        val on = Timer.getFPGATimestamp() % (duration * 2) < duration
        solid(if (on) color1 else color2)
    }

    private fun stripes(vararg stripes: Stripe) {
        stripes(null, *stripes)
    }

    private fun stripes(durationPerStripe: Double?, vararg stripes: Stripe) {
        val stripeSize = length / stripes.size
        val offset =
            // toInt floors if positive, ceils if negative. It should always be positive
            if (durationPerStripe != null) (Timer.getFPGATimestamp() / durationPerStripe * stripeSize).toInt()
            else 0
        var start = 0
        for ((i, stripe) in stripes.withIndex()) {
            val end = stripeSize * (i + 1)
            for (j in start..<end) {
                val color =
                    if (stripe.blinkDuration != null && Timer.getFPGATimestamp() % (stripe.blinkDuration * 2) > stripe.blinkDuration) Color.kBlack
                    else stripe.color
                buffer.setLED((j + offset) % length, color)
            }
            start = end
        }
    }

    data class Stripe(val color: Color, val blinkDuration: Double? = null)

    private fun rainbow(totalDuration: Double, backwards: Boolean = false) {
        val offset = (Timer.getFPGATimestamp() % totalDuration / totalDuration * length).toInt()
        val range = 0..<length
        for ((i, j) in if (backwards) range.withIndex() else range.reversed().withIndex()) {
            buffer.setHSV(i, 180 * (j + offset % length) / length, 255, 255)
        }
    }

    private fun switchBetween(durationPer: Double, vararg functions: () -> Unit) {
        val t = Timer.getFPGATimestamp() % (durationPer * functions.size)
        try {
            functions[(t / durationPer).toInt()].invoke()
        } catch (e: IndexOutOfBoundsException) {
            println("Couldn't switch to function")
            e.printStackTrace()
            functions.firstOrNull()?.invoke()
        }
    }
}