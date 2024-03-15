package frc.robot

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.*

object Constants {
    /**
     * Automatically determined based on if code is running on a real robot and if [Simulation.shouldReplay] is enabled
     */
    val mode = if (RobotBase.isReal()) Mode.REAL else (if (Simulation.shouldReplay) Mode.REPLAY else Mode.SIM)

    val useFileConstants = mode == Mode.SIM

    const val driverControllerPort = 0
    const val operatorControllerPort = 1

    const val controllerDeadzone = 0.05
    const val useControllerDeadzone = true

    const val pdhId = 12

    const val errorRumbleAmount = 0.75
    const val errorRumbleDuration = 0.25

    enum class Mode {
        /**
         * Real robot
         */
        REAL,

        /**
         * Simulation
         */
        SIM,

        /**
         * Log replay
         */
        REPLAY,
    }

    object Climber {
        const val rightMotorId = 7
        const val leftMotorId = 8

        const val gearRatio = 35.0
        const val spoolDiameter = 0.75
        const val ropeLength = 25.5
        const val ropeLeftThreshold = 1.0
    }

    object Drivebase {
        const val leftLeaderMotorId = 1
        const val leftFollowerMotorId = 2
        const val rightLeaderMotorId = 4
        const val rightFollowerMotorId = 3
        const val pigeonId = 9

        val wheelSize = KitbotWheelSize.kSixInch
        val wheelRadius = wheelSize.value / 2 // const value is diameter, we need radius
        val trackWidth = Units.inchesToMeters(26.0)
        val gearRatio = KitbotGearing.k10p71
        val motor = KitbotMotor.kDualCIMPerSide

        const val leftEncoderChannelA = 5
        const val leftEncoderChannelB = 7
        const val rightEncoderChannelA = 2
        const val rightEncoderChannelB = 1
        const val encoderDistancePerPulse = 1 / 2048.0 // AKA divide encoder reading by 2048

        const val velocityP = 1.2
        const val velocityD = 0.0

        val feedforwardLeftS = ifSimElse(0.7, 1.0)
        val feedforwardLeftV = ifSimElse(2.0, 2.2)
        val feedforwardRightS = ifSimElse(feedforwardLeftS, 1.0)
        val feedforwardRightV = ifSimElse(feedforwardLeftV, 2.2)

        val swerveModeDeadzone = if (Simulation.useNintendoSwitchProController) 0.5 else 0.8
        val swerveModeP = ifSimElse(0.08, 0.05)
        val swerveModeD = ifSimElse(0.001, 0.008)

        const val pathfindMaxSpeed = 1.0 //2.5;
        const val pathfindMaxAccel = pathfindMaxSpeed
        const val pathfindEndSpeed = 0.7
    }

    object Intake {
        const val pivotMotorId = 9
        const val driverMotorId = 10

        const val pivotGearRatio = 25.0
        val pivotRadDown = Units.degreesToRadians(195.0)
        val pivotRadEject = pivotRadDown - Units.degreesToRadians(80.0)
        const val pivotP = 2.5
        const val pivotD = 0.0
        val pivotFFg = ifSimElse(0.01, 0.35)

        const val intakeSpeed = 1.0
        const val handoffSpeed = -0.75
        const val handoffTimeout = 0.15
        const val ejectSpeed = -1.0
        const val ejectTimeout = 0.5
        const val ejectIntakeTimeout = 0.15
    }

    object Launcher {
        const val topMotorId = 5
        const val bottomMotorId = 6

        const val launchingSpeed = 1.0

        const val topIntakeSpeed = -1.0
        const val bottomIntakeSpeed = -0.2

        const val spinUpTime = 0.3
    }

    object Simulation {
        const val shouldReplay = false
        val useNintendoSwitchProController =
            RobotBase.isSimulation() && System.getProperty("os.name").contains("Mac OS X")
    }
}