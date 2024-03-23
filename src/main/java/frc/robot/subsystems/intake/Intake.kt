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
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.subsystems.controller.DriverController
import frc.robot.subsystems.leds.withLEDs
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
    private val mechanism = kotlin.run {
        val mechanism = Mechanism2d(6.0, 6.0, Color8Bit(Color.kGray))
        SmartDashboard.putData("Intake", mechanism)
        mechanism
    }
    private val mechanismLigament = kotlin.run {
        val root = mechanism.getRoot("Root", 3.0, 3.0)
        root.append(MechanismLigament2d("Pivot", 3.0, 0.0, 4.0, Color8Bit(Color.kOrange)))
    }
    private var usePivotPID = true

    private val manualIntaking = LoggedDashboardBoolean("Manual intaking", false)

    init {
        Trigger { inputs.hasNote }
            .onTrue(DriverController.setRumble(0.5, 0.5))
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Inputs/Intake", inputs)

        mechanismLigament.angle = Units.radiansToDegrees(inputs.pivotPositionRad + Constants.Intake.pivotRadDown)
        Logger.recordOutput("Intake/Mechanism", mechanism)

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
        return Commands.sequence(
            pivotPIDToCommand(-Constants.Intake.pivotRadDown),
            startEnd(
                { io.setDriverVoltage(Constants.Intake.intakeSpeed * 12) },
                { io.stopDriver() }
            ).until { inputs.hasNote && !manualIntaking.get() },
        )
            .withLEDs(Color.kYellow, Color.kGreen)
            .andThen(tuckCommand())
            .withName("Intake\$intake")
    }

    fun tuckCommand(): Command {
        return pivotPIDToCommand(0.0).withName("Intake\$tuck")
    }

    fun handoffCommand(): Command {
        return Commands.sequence(
            tuckCommand(),
            startEnd(
                { io.setDriverVoltage(Constants.Intake.handoffSpeed * 12) },
                { io.stopDriver() }
            ).withTimeout(Constants.Intake.handoffTimeout)
        )
            .withLEDs(Color.kBlue, Color.kBlueViolet)
            .withName("Intake\$handoff")
    }

    fun ejectCommand(): Command {
        return Commands.sequence(
            startEnd(
                { io.setDriverVoltage(Constants.Intake.intakeSpeed * 12) },
                { io.stopDriver() }
            ).withTimeout(Constants.Intake.ejectIntakeTimeout),
            pivotPIDToCommand(-Constants.Intake.pivotRadEject),
            startEnd(
                { io.setDriverVoltage(Constants.Intake.ejectSpeed * 12) },
                { io.stopDriver() }
            ).withTimeout(Constants.Intake.ejectTimeout),
        )
            .withLEDs(Color.kOrange, Color.kGreen)
            .andThen(tuckCommand())
            .withName("Intake\$eject")
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
        )
            .withLEDs(Color.kDarkRed, null)
            .withName("Intake\$pivotSlightlyDown")
    }

    fun resetPivotCommand(): Command {
        return runOnce { io.resetPivotPosition() }
            .withLEDs(null, Color.kDarkRed)
    }
}
