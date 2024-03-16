package frc.robot.subsystems.drivebase

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.*
import frc.robot.commands.FeedforwardCharacterization
import frc.robot.subsystems.controller.DriverController
import frc.robot.commands.AutoAlign
import frc.robot.commands.SwerveMode
import frc.robot.util.LocalADStarAK
import frc.robot.util.TunablePIDController
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import java.util.function.Supplier
import kotlin.math.abs

object Drivebase : SubsystemBase() {
    private val io = switchMode(::DrivebaseIOReal, ::DrivebaseIOSim, ::DrivebaseIO)
    private val inputs = DrivebaseIO.inputs()

    private val gyroIO = switchMode(::GyroIOReal, ::GyroIOSim, ::GyroIO)
    private val gyroInputs = GyroIO.inputs()

    private val limelightIO = ifRealElse(::LimelightIOReal, ::LimelightIO)
    private val limelightInputs = LimelightIO.inputs()

    private val kinematics = DifferentialDriveKinematics(Constants.Drivebase.trackWidth)
    private val odometry = DifferentialDrivePoseEstimator(kinematics, Rotation2d(), 0.0, 0.0, Pose2d())
    private val field = Field2d()
    private val leftFeedforward =
        SimpleMotorFeedforward(Constants.Drivebase.feedforwardLeftS, Constants.Drivebase.feedforwardLeftV)
    private val rightFeedforward =
        SimpleMotorFeedforward(Constants.Drivebase.feedforwardRightS, Constants.Drivebase.feedforwardRightV)
    private val driveVelocityPID = TunablePIDController(
        "Drivebase driveVelocity",
        Constants.Drivebase.velocityP,
        0.0,
        Constants.Drivebase.velocityD
    )
    private var lastLeftPositionMeters = 0.0
    private var lastRightPositionMeters = 0.0
    private var yaw = Rotation2d()

    private val usePoseEstimation = LoggedDashboardBoolean("Use Pose Estimation", true)
    private val fallbackRotationRevert = LoggedDashboardBoolean("Fallback to rotation reverting", false)
    private val disableDriving = LoggedDashboardBoolean("Disable Driving", false)
    private val arcadeDriveToggle = LoggedDashboardBoolean("Arcade Drive", false)

    private var arcadeDrive = arcadeDriveToggle.get()

    @AutoLogOutput
    var reverseMode = false
        private set

    init {
        SmartDashboard.putData("Field", field)

        AutoBuilder.configureLTV(
            this::pose,
            { pose ->
                odometry.resetPosition(
                    yaw,
                    leftPositionMeters,
                    rightPositionMeters,
                    pose
                )
            },
            {
                kinematics.toChassisSpeeds(
                    DifferentialDriveWheelSpeeds(
                        leftVelocityMetersPerSec,
                        rightVelocityMetersPerSec
                    )
                )
            },
            { speeds ->
                val wheelSpeeds = kinematics.toWheelSpeeds(speeds)
                driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond)
            },
            0.02,
            ReplanningConfig(),
            ::shouldFlip,
            this
        )
        Pathfinding.setPathfinder(LocalADStarAK())
        PathPlannerLogging.setLogActivePathCallback { activePath ->
            Logger.recordOutput(
                "Drivebase/Trajectory",
                *activePath.toTypedArray<Pose2d>()
            )
        }
        PathPlannerLogging.setLogTargetPoseCallback { targetPose ->
            Logger.recordOutput(
                "Drivebase/TrajectorySetpoint",
                targetPose
            )
        }

        defaultCommand = SwerveMode.swerveDriveCommand()
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Inputs/Drivebase", inputs)

        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Inputs/Gyro", gyroInputs)

        limelightIO.updateInputs(limelightInputs)
        Logger.processInputs("Inputs/Limelight", limelightInputs)

        val leftPositionMeters = leftPositionMeters
        val rightPositionMeters = rightPositionMeters

        if (gyroInputs.connected) {
            yaw = gyroInputs.yaw
        } else {
            val twist2d = kinematics.toTwist2d(
                leftPositionMeters - lastLeftPositionMeters,
                rightPositionMeters - lastRightPositionMeters
            )
            yaw = yaw.plus(Rotation2d(twist2d.dtheta))
        }

        lastLeftPositionMeters = leftPositionMeters
        lastRightPositionMeters = rightPositionMeters

        field.robotPose = odometry.update(yaw, leftPositionMeters, rightPositionMeters)

        if (usePoseEstimation.get()) {
            if (limelightInputs.leftTv == 1L) addVisionMeasurement(
                limelightInputs.leftBotpose,
                limelightInputs.leftBotposeTimestamp,
                limelightInputs.leftTagCount,
                limelightInputs.leftAvgArea
            )
            if (limelightInputs.rightTv == 1L) addVisionMeasurement(
                limelightInputs.rightBotpose,
                limelightInputs.rightBotposeTimestamp,
                limelightInputs.leftTagCount,
                limelightInputs.leftAvgArea
            )
        } else if (fallbackRotationRevert.get()) {
            // Revert rotation changes
            odometry.addVisionMeasurement(
                Pose2d(pose.x, pose.y, gyroInputs.yaw),
                Timer.getFPGATimestamp(),
                VecBuilder.fill(0.0, 0.0, 0.0)
            )
        }

        if (arcadeDriveToggle.get() != arcadeDrive) {
            arcadeDrive = arcadeDriveToggle.get()
            updateDefaultCommand()
        }
    }

    fun teleopInit() {
        usePoseEstimation.set(false)
        gyroIO.setYaw(pose.rotation)
        odometry.addVisionMeasurement( // Rotation must be zero because gyro will take over
            // if we use the real rotation it will be doubled due to setting gyro to it
            Pose2d(pose.x, pose.y, Rotation2d()),
            Timer.getFPGATimestamp(),
            VecBuilder.fill(0.0, 0.0, 0.0)
        )
    }

    private fun addVisionMeasurement(botpose: Pose2d, timestamp: Double, tagCount: Double, avgArea: Double) {
        val odometryDifference = odometry.estimatedPosition.translation.getDistance(botpose.translation)

        var xyStdDev: Double
        var rotStdDev: Double

        if (avgArea > 0.8 && odometryDifference < 0.5) {
            xyStdDev = 1.0
            rotStdDev = 10.0
        } else if (avgArea > 0.8) {
            xyStdDev = 1.5
            rotStdDev = 10.0
        } else if (avgArea > 0.5 && odometryDifference < 1) {
            xyStdDev = 2.0
            rotStdDev = 15.0
        } else if (avgArea > 0.2 && odometryDifference < 2) {
            xyStdDev = 4.0
            rotStdDev = 30.0
        } else if (avgArea > 0.05 && odometryDifference < 5) {
            xyStdDev = 10.0
            rotStdDev = 30.0
        } else return

        if (tagCount >= 2) {
            xyStdDev -= if (avgArea > 0.8) 0.25 else 0.5
            rotStdDev -= 8.0
        }

        odometry.addVisionMeasurement(
            botpose,
            timestamp,
            VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(rotStdDev))
        )
    }

    /**
     * @param speed    Positive = forward
     * @param rotation Positive = counterclockwise
     */
    fun arcadeDrive(speed: Double, rotation: Double) {
        Logger.recordOutput("Drivebase/ArcadeDrive/Speed", speed)
        Logger.recordOutput("Drivebase/ArcadeDrive/Rotation", rotation)
        val speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, false)
        if (!disableDriving.get()) io.setVoltage(speeds.left * 12, speeds.right * 12)
    }

    private fun driveVelocity(leftMetersPerSec: Double, rightMetersPerSec: Double) {
        Logger.recordOutput("Drivebase/DriveVelocity/LeftSetpointMetersPerSec", leftMetersPerSec)
        Logger.recordOutput("Drivebase/DriveVelocity/RightSetpointMetersPerSec", rightMetersPerSec)

        val leftPID = driveVelocityPID.calculate(leftVelocityMetersPerSec, leftMetersPerSec)
        val rightPID = driveVelocityPID.calculate(rightVelocityMetersPerSec, rightMetersPerSec)
        Logger.recordOutput("Drivebase/DriveVelocity/LeftControlSignalPID", leftPID)
        Logger.recordOutput("Drivebase/DriveVelocity/RightControlSignalPID", rightPID)

        val leftFF = leftFeedforward.calculate(leftMetersPerSec)
        val rightFF = rightFeedforward.calculate(rightMetersPerSec)
        Logger.recordOutput("Drivebase/DriveVelocity/LeftControlSignalFF", leftFF)
        Logger.recordOutput("Drivebase/DriveVelocity/RightControlSignalFF", rightFF)

        io.setVoltage(leftPID + leftFF, rightPID + rightFF)
    }

    fun driveVelocityCommand(leftMetersPerSec: Double, rightMetersPerSec: Double): Command {
        return this.run { driveVelocity(leftMetersPerSec, rightMetersPerSec) }.withName("Drivebase\$driveVelocity")
    }

    private fun arcadeDriveCommand(): Command {
        return run {
            val reverse = if (reverseMode) -1 else 1
            var speed = reverse * DriverController.speed()
            var rotation = -DriverController.leftX

            if (Constants.useControllerDeadzone) {
                if (abs(speed) < Constants.controllerDeadzone) speed = 0.0
                if (abs(rotation) < Constants.controllerDeadzone) rotation = 0.0
            }
            arcadeDrive(speed, rotation)
        }.withName("Drivebase\$arcadeDrive")
    }

    fun followPathCommand(name: String): Command {
        val path = PathPlannerPath.fromPathFile(name)
        return setPoseCommand(flipIfNeeded(path.startingDifferentialPose)).andThen(AutoBuilder.followPath(path))
            .withName("Drivebase\$followPath")
    }

    fun pathfindCommand(targetPoseSupplier: Supplier<Pose2d>): Command {
        return Commands.deferredProxy {
            val pose = pose
            val targetPose = targetPoseSupplier.get()

            // Check if the robot is already at the target pose.
            if (abs(pose.x - targetPose.x) <= .1 && abs(pose.y - targetPose.y) <= .1) {
                return@deferredProxy Commands.none()
            }

            val bezierPoints = PathPlannerPath.bezierFromPoses(
                pose,
                targetPose
            )

            val path = PathPlannerPath(
                bezierPoints,
                PathConstraints(
                    Constants.Drivebase.pathfindMaxSpeed,
                    Constants.Drivebase.pathfindMaxAccel,
                    2 * Math.PI,
                    4 * Math.PI
                ),  // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                GoalEndState(
                    Constants.Drivebase.pathfindEndSpeed,
                    targetPose.rotation
                ) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            )

            path.preventFlipping = true
            AutoBuilder
                .followPath(path)
                .andThen(SwerveMode.swerveAngleCommand(targetPose.rotation.degrees))
        }.withName("Drivebase\$pathfind")
    }

    fun toggleReverseModeCommand(): Command {
        return Commands.runOnce({ reverseMode = !reverseMode })
    }

    fun toggleArcadeDrive(): Command {
        return Commands.runOnce({
            val newVal = !arcadeDriveToggle.get()
            arcadeDriveToggle.set(newVal)
            arcadeDrive = newVal
            updateDefaultCommand()
        })
    }

    fun updateDefaultCommand() {
        println("Updating default command (arcadeDrive = $arcadeDrive)")
        if (this.currentCommand === this.defaultCommand) this.currentCommand.cancel()
        if (arcadeDrive) {
            this.defaultCommand = arcadeDriveCommand()
        } else {
            this.defaultCommand = SwerveMode.swerveDriveCommand()
        }
    }

    fun feedforwardCharacterizationLeft(): Command {
        return FeedforwardCharacterization(this, { volts ->
            io.setVoltage(volts, volts)
        }, this::leftVelocityMetersPerSec)
    }

    fun feedforwardCharacterizationRight(): Command {
        return FeedforwardCharacterization(this, { volts ->
            io.setVoltage(volts, volts)
        }, this::rightVelocityMetersPerSec)
    }

    @get:AutoLogOutput
    val pose: Pose2d
        get() = odometry.estimatedPosition

    fun setPoseCommand(newPose: Pose2d): Command {
        return setPoseCommand { newPose }
    }

    fun setPoseCommand(newPose: Supplier<Pose2d>): Command {
        return Commands.runOnce({ odometry.resetPosition(yaw, leftPositionMeters, rightPositionMeters, newPose.get()) })
    }

    val gyro: Rotation2d
        get() = gyroInputs.yaw

    fun resetGyroCommand(): Command {
        return runOnce {
            gyroIO.setYaw(Rotation2d())
            yaw = Rotation2d()
        }.andThen(SwerveMode.swerveAngleCommand(0.0))
    }

    @get:AutoLogOutput
    val leftPositionMeters: Double
        get() = inputs.leftPositionRad * Constants.Drivebase.wheelRadius

    @get:AutoLogOutput
    val rightPositionMeters: Double
        get() = inputs.rightPositionRad * Constants.Drivebase.wheelRadius

    @get:AutoLogOutput
    val leftVelocityMetersPerSec: Double
        get() = inputs.leftVelocityRadPerSec * Constants.Drivebase.wheelRadius

    @get:AutoLogOutput
    val rightVelocityMetersPerSec: Double
        get() = inputs.rightVelocityRadPerSec * Constants.Drivebase.wheelRadius
}
