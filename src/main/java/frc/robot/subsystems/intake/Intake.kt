package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotState
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.leds.LEDs
import frc.robot.switchMode
import frc.robot.util.TunablePIDController
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean

object Intake : SubsystemBase() {
    private val io = switchMode(::IntakeIOReal, ::IntakeIOSim, ::IntakeIO)
    private val inputs = IntakeIO.inputs()

    private val pivotPID = kotlin.run {
        val p = TunablePIDController("Intake: Pivot", Constants.Intake.pivotP, 0.0, Constants.Intake.pivotD)
        p.setTolerance(Units.degreesToRadians(15.0))
        p
    }
    private val pivotFF = ArmFeedforward(0.0, Constants.Intake.pivotFFg, 0.0, 0.0)
    private val pivotMechanism = kotlin.run {
        val mechanism = Mechanism2d(6.0, 6.0, Color8Bit(Color.kGray))
        SmartDashboard.putData("Intake", mechanism)
        val root = mechanism.getRoot("Root", 3.0, 3.0)
        root.append(MechanismLigament2d("Pivot", 3.0, 0.0, 4.0, Color8Bit(Color.kOrange)))
    }
    private var usePivotPID = true

    private val manualIntaking = LoggedDashboardBoolean("Manual intaking", false)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Inputs/Intake", inputs)

        pivotMechanism.angle =
            Units.radiansToDegrees(inputs.pivotPositionRad + Constants.Intake.pivotRadDown)

        val pid = pivotPID.calculate(inputs.pivotPositionRad)
        val ff = pivotFF.calculate(
            inputs.pivotPositionRad - Constants.Intake.pivotRadDown,
            inputs.pivotVelocityRadPerSec
        )
        Logger.recordOutput("Intake/PivotControlSignalPID", pid)
        Logger.recordOutput("Intake/PivotControlSignalFF", ff)
        if (RobotState.isEnabled() && usePivotPID) io.setPivotVoltage(pid + ff)
    }

    private fun pivotPIDToCommand(setpoint: Double): Command {
        return startEnd({ pivotPID.setpoint = setpoint }, {}).until { pivotPID.atSetpoint() }
    }

    fun intakeCommand(): Command {
        return pivotPIDToCommand(-Constants.Intake.pivotRadDown).andThen(
            startEnd(
                { io.setDriverVoltage(Constants.Intake.intakeSpeed * 12) },
                { io.stopDriver() }
            ).raceWith(Commands.waitUntil { inputs.hasNote && !manualIntaking.get() })
        )
            .raceWith(LEDs.blinkCommand(Color.kYellow, Constants.LEDs.blinkDurationInProgress))
            .andThen(
                tuckCommand().alongWith(
                    LEDs.blinkCommand(Color.kGreen, Constants.LEDs.blinkDurationCompleted).withTimeout(1.0)
                )
            ) // TODO rumble driver here
            .withName("Intake\$intake")
    }

    fun tuckCommand(): Command {
        return pivotPIDToCommand(0.0).withName("Intake\$tuck")
    }

    fun handoffCommand(): Command {
        return tuckCommand().andThen(
            startEnd(
                { io.setDriverVoltage(Constants.Intake.handoffSpeed * 12) },
                { io.stopDriver() }
            ).withTimeout(Constants.Intake.handoffTimeout)
        ).withName("Intake\$handoff")
    }

    fun ejectCommand(): Command {
        return Commands.sequence(
            Commands.sequence(
                startEnd(
                    { io.setDriverVoltage(Constants.Intake.intakeSpeed * 12) },
                    { io.stopDriver() }
                ).withTimeout(Constants.Intake.ejectIntakeTimeout),
                pivotPIDToCommand(-Constants.Intake.pivotRadEject),
                startEnd(
                    { io.setDriverVoltage(Constants.Intake.ejectSpeed * 12) },
                    { io.stopDriver() }
                ).withTimeout(Constants.Intake.ejectTimeout),
            ).raceWith(LEDs.blinkCommand(Color.kOrange, Constants.LEDs.blinkDurationInProgress)),
            tuckCommand().alongWith(LEDs.blinkCommand(Color.kGreen, Constants.LEDs.blinkDurationCompleted).withTimeout(1.0))
        ).withName("Intake\$eject")
    }

    fun pivotSlightlyDownCommand(): Command {
        return startEnd(
            {
                usePivotPID = false
                io.setPivotVoltage(1.0)
            },
            {
                usePivotPID = true
            }
        ).withName("Intake\$pivotSlightlyDown")
    }

    fun resetPivotCommand(): Command {
        return runOnce { io.resetPivotPosition() }
    }
}
