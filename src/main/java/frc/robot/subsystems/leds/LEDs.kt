package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color.*
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WrapperCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.Constants.LEDs.length
import frc.robot.commands.Actions
import frc.robot.subsystems.drivebase.Drivebase
import frc.robot.subsystems.leds.patterns.*
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean

object LEDs : SubsystemBase() {
    private val buffer = AddressableLEDBuffer(length)
    private val led = kotlin.run {
        val led = AddressableLED(Constants.LEDs.id)
        led.setLength(length)
        led.setData(buffer)
        led.start()
        led
    }
    internal var currentPattern: LEDPattern = Solid(kBlack)

    private const val sideHeight = length / 2
    private val mechanism = kotlin.run {
        val mechanism = Mechanism2d(6.0, sideHeight.toDouble(), Color8Bit(kGray))
        if (Constants.LEDs.debugMechanism)
            SmartDashboard.putData("LEDs", mechanism)
        mechanism
    }
    private val mechanisms = kotlin.run {
        val mechanisms: ArrayList<MechanismLigament2d> = ArrayList()
        if (Constants.LEDs.debugMechanism)
            for (i in 0..<length) {
                val x = if (i < sideHeight) 1.0 else 5.0
                val y = if (i < sideHeight) i.toDouble() else sideHeight - i.toDouble() % sideHeight
                val root = mechanism.getRoot(i.toString(), x, y)
                mechanisms.add(
                    root.append(
                        MechanismLigament2d(
                            i.toString(),
                            1.0,
                            if (i < sideHeight) 90.0 else -90.0,
                            14.0,
                            Color8Bit(kBlack)
                        )
                    )
                )
            }
        mechanisms
    }

    val kPink = Color(255, 0, 128)

    init {
        defaultCommand = run {
            if (DriverStation.isEStopped()) estopped()
            else if (DriverStation.isAutonomousEnabled()) autonomous()
            else if (DriverStation.isTeleopEnabled()) teleop()
            else disabled()
        }.ignoringDisable(true).withName("LEDs\$passive")

        Trigger { RobotController.getBatteryVoltage() <= Constants.LEDs.lowBatteryVoltsThreshold }
            .debounce(5.0)
            .onTrue(Commands.runOnce({ lowBattery = true }))
            .onFalse(Commands.runOnce({ lowBattery = false }))
        Trigger { DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() < Constants.LEDs.endgameThreshold }
            .onTrue(Commands.startEnd({ endgameAlert = true }, { endgameAlert = false }).withTimeout(5.0))
        // autoFinished is set in CommandRobot#autonomousPeriodic for simplicity
    }

    override fun periodic() {
        currentPattern.periodic(0..<length, BufferWrapper(buffer), Timer.getFPGATimestamp())

        if (Constants.LEDs.debugMechanism) {
//            for (i in 0..<length)
//                print("\u001b[38;2;${buffer.getRed(i)};${buffer.getGreen(i)};${buffer.getBlue(i)}m■\u001b[0m")
//            println()
            for (i in 0..<length)
                mechanisms.getOrNull(i)?.color = Color8Bit(buffer.getLED(i))
            Logger.recordOutput("LEDs/Mechanism", mechanism)
        }

        led.setData(buffer)
    }

    var autoFinished = false
    var lowBattery = false
    var endgameAlert = false
    val prideWhileDisabled = LoggedDashboardBoolean("Pride LEDs while disabled", false)

    private val lowBatteryPattern = Blink(Solid(kRed), 0.5)
    private val endgameAlertPattern = Blink(Solid(kYellow), 0.25)

    private fun estopped() {
        currentPattern = Solid(kRed)
    }

    private fun disabled() {
        if (lowBattery) {
            currentPattern = lowBatteryPattern
            return
        }

        if (prideWhileDisabled.get()) {
            currentPattern = Stripes(
                listOf(
                    Solid(kWhite),
                    Solid(kPink),
                    Solid(kCyan),
                    Solid(Color(162, 85, 42)),
                    Solid(kBlack),
                    Solid(kRed),
                    Solid(kOrange),
                    Solid(kYellow),
                    Solid(kGreen),
                    Solid(kBlue),
                    Solid(kPurple),
                ),
                0.2
            )
            return
        }

        currentPattern = Switching(
            listOf(
                gradientBothSides(kRed, kPink, 2.0),
                rainbowBothSides(1.5),
            ),
            10.0
        )
    }

    private fun autonomous() {
        currentPattern =
            if (autoFinished) Solid(kGreen)
            else {
                val pattern = Wave(
                    kWhite,
                    DriverStation.getAlliance()
                        .map { alliance ->
                            if (alliance == DriverStation.Alliance.Red) kRed
                            else kBlue
                        }
                        .orElse(kGold),
                    0.5
                )
                Split(listOf(pattern.backwards(), pattern))
            }
    }

    private fun teleop() {
        val reversePattern = Solid(if (Drivebase.reverseMode) kRed else kGreen)
        val actionPattern = Solid(
            when (Actions.selectedAction) {
                Actions.Action.None -> kBlack
                Actions.Action.Source -> kOrange
                Actions.Action.FrontSubwoofer, Actions.Action.LeftSubwoofer, Actions.Action.RightSubwoofer -> kAqua
            }
        )
        if (endgameAlert)
            currentPattern = Split(
                listOf(
                    reversePattern,
                    // split the endgame alert between each side of the LEDs
                    endgameAlertPattern,
                    endgameAlertPattern,
                    actionPattern,
                )
            )
        else
            currentPattern = Split(
                listOf(
                    reversePattern,
                    actionPattern
                )
            )
    }

    fun startLoadingNotifier(): Notifier {
        val notifier = Notifier {
            Fade(kLightPink, kBlack, 1.0).periodic(
                0..<length,
                BufferWrapper(buffer),
                length,
                System.currentTimeMillis() / 1000.0
            )
            led.setData(buffer)
        }
        notifier.startPeriodic(0.02)
        return notifier
    }

    fun setPatternCommand(pattern: LEDPattern): Command {
        return startEnd({ currentPattern = pattern }, {})
    }
}

fun Command.withLEDs(
    colorInProgress: Color?,
    colorComplete: Color?,
    showCompletedIfInterrupted: Boolean = false
): Command {
    return withLEDs(
        colorInProgress?.let { Blink(Solid(it), Constants.LEDs.blinkDurationInProgress) },
        colorComplete?.let { Blink(Solid(it), Constants.LEDs.blinkDurationCompleted) },
        false
    )
}

fun Command.withLEDs(
    patternInProgress: LEDPattern?,
    patternComplete: LEDPattern?,
    showCompletedIfInterrupted: Boolean = false
): Command {
    return object : WrapperCommand(this) {
        private val inProgressCommand =
            patternInProgress?.let {
                LEDs.setPatternCommand(it)
                    .withName("LEDs\$instrument\$InProgress")
            }
        private val completeCommand =
            patternComplete?.let {
                LEDs.setPatternCommand(it)
                    .withTimeout(1.0)
                    .withName("LEDs\$instrument\$Complete")
            }

        override fun initialize() {
            super.initialize()

            inProgressCommand?.schedule()
        }

        override fun end(interrupted: Boolean) {
            super.end(interrupted)

            if (showCompletedIfInterrupted || !interrupted)
                completeCommand?.schedule()
            if (inProgressCommand?.isScheduled == true)
                inProgressCommand.cancel()
        }
    }
}

/**
 * A wrapper for [AddressableLEDBuffer] that automatically wraps the index parameters to fit the buffer.
 * **It does not wrap index parameters to fit the current pattern - you need to do this yourself.**
 */
class BufferWrapper(private val buffer: AddressableLEDBuffer) {
    /**
     * Sets a specific led in the buffer.Automatically wraps index to fit the buffer.
     * **Does not wrap index to fit the current pattern - you need to do this yourself.**
     *
     * @param index the index to write
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    fun setRGB(index: Int, r: Int, g: Int, b: Int) {
        buffer.setRGB(index % length, r, g, b)
    }

    /**
     * Sets a specific led in the buffer. Automatically wraps index to fit the buffer.
     * **Does not wrap index to fit the current pattern - you need to do this yourself.**
     *
     * @param index the index to write
     * @param h the h value [0-180)
     * @param s the s value [0-255]
     * @param v the v value [0-255]
     */
    fun setHSV(index: Int, h: Int, s: Int, v: Int) {
        buffer.setHSV(index % length, h, s, v)
    }

    /**
     * Sets a specific LED in the buffer. Automatically wraps index to fit the buffer.
     * **Does not wrap index to fit the current pattern - you need to do this yourself.**
     *
     * @param index The index to write
     * @param color The color of the LED
     */
    fun setLED(index: Int, color: Color) {
        buffer.setLED(index % length, color)
    }
}