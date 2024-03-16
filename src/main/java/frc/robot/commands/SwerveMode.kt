package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.subsystems.controller.DriverController
import frc.robot.subsystems.drivebase.Drivebase
import frc.robot.util.TunablePIDController
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.atan2

object SwerveMode {
    private val swerveModePID = run {
        val pid = TunablePIDController(
            "Swerve Mode",
            Constants.Drivebase.swerveModeP,
            0.0,
            Constants.Drivebase.swerveModeD
        )
        pid.enableContinuousInput(-180.0, 180.0)
        pid.setTolerance(3.0)
        pid
    }

    fun swerveDriveCommand(): Command {
        return SwerveDriveCommand()
    }

    fun swerveAngleCommand(angle: Double): Command {
        return Commands.runOnce({ swerveModePID.setpoint = angle })
    }

    fun swerveToAngleCommand(angle: Double): Command {
        return swerveAngleCommand(angle).andThen(swerveDriveCommand()).until { swerveModePID.atSetpoint() }
    }

    private class SwerveDriveCommand : Command() {
        init {
            addRequirements(Drivebase)
            name = "Drivebase\$swerveDrive"
        }

        override fun initialize() {
            swerveModePID.setpoint = Drivebase.gyro.degrees
        }

        override fun execute() {
            val reverse = if (Drivebase.reverseMode) -1 else 1

            val x = reverse * DriverController.leftX
            val y = reverse * -DriverController.leftY

            if (abs(x) > Constants.Drivebase.swerveModeDeadzone || abs(y) > Constants.Drivebase.swerveModeDeadzone) {
                swerveModePID.setpoint = -Math.toDegrees(atan2(x, y))
            }

            Logger.recordOutput("Drivebase/SwerveMode/Setpoint", swerveModePID.setpoint)

            val robotAngle = Drivebase.gyro.degrees
            Logger.recordOutput("Drivebase/SwerveMode/Measurement", robotAngle)

            var speed = reverse * DriverController.speed()
            val rotation = swerveModePID.calculate(robotAngle)

            if (Constants.useControllerDeadzone) {
                if (abs(speed) < Constants.controllerDeadzone) speed = 0.0
            }

            Drivebase.arcadeDrive(speed, rotation)
        }
    }
}
