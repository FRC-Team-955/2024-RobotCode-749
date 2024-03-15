@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package frc.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.lang.reflect.Array

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object CommandRobot : LoggedRobot() {
    private var autonomousCommand: Command? = null

    private fun logConstantClass(clazz: Class<*>, parentName: String?) {
        val parent = (if (parentName != null) "$parentName." else "")
        for (field in clazz.fields) {
            val key = "$parent${clazz.simpleName}.${field.name}"
            try {
                val value = field[null]
                if (value.javaClass.isArray) {
                    for (i in 0 until Array.getLength(value)) {
                        Logger.recordMetadata("$key[$i]", Array.get(value, i).toString())
                    }
                } else {
                    Logger.recordMetadata(key, value.toString())
                }
            } catch (e: Exception) {
                Logger.recordMetadata(key, "Unknown")
            }
        }
        for (subclass in clazz.classes) {
            logConstantClass(subclass, parent + clazz.simpleName)
        }
    }

    override fun robotInit() {
        Logger.recordMetadata("* ProjectName", BuildConstants.MAVEN_NAME)
        Logger.recordMetadata("* BuildDate", BuildConstants.BUILD_DATE)
        Logger.recordMetadata("* GitSHA", BuildConstants.GIT_SHA)
        Logger.recordMetadata("* GitDate", BuildConstants.GIT_DATE)
        Logger.recordMetadata("* GitBranch", BuildConstants.GIT_BRANCH)
        when (BuildConstants.DIRTY) {
            0 -> Logger.recordMetadata("* GitDirty", "All changes committed")
            1 -> Logger.recordMetadata("* GitDirty", "Uncommitted changes")
            else -> Logger.recordMetadata("* GitDirty", "Unknown")
        }

        // By doing this, we also initialize file constants and any other variable constants
        logConstantClass(Constants::class.java, null)

        when (Constants.mode) {
            Constants.Mode.REAL -> {
                Logger.addDataReceiver(WPILOGWriter()) // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(NT4Publisher()) // Log to NetworkTables
                // Enables power distribution logging
                SmartDashboard.putData(
                    "PowerDistribution",
                    PowerDistribution(Constants.pdhId, PowerDistribution.ModuleType.kRev)
                )
            }

            Constants.Mode.SIM -> {
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.Mode.REPLAY -> {
                setUseTiming(false) // Run as fast as possible
                // Pull the replay log from AdvantageScope (or prompt the user)
                val logPath = LogFileUtil.findReplayLog()
                // Read replay log
                Logger.setReplaySource(WPILOGReader(logPath))
                // Save outputs to a new log
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }

        Logger.start()

        // Report the use of the Kotlin Language for "FRC Usage Report" statistics
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language,
            FRCNetComm.tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version
        )

        // Access the Robot object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        registerFieldsForAutoLogOutput(Robot)
    }


    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        autonomousCommand = Robot.getAutonomousCommand()
        autonomousCommand?.schedule()
    }

    override fun autonomousExit() {
        autonomousCommand?.cancel()
    }

    override fun teleopInit() {
        Robot.teleopInit()
    }
}

/**
 * Main initialization function. Do not perform any initialization here
 * other than calling `RobotBase.startRobot`. Do not modify this file
 * except to change the object passed to the `startRobot` call.
 *
 * If you change the package of this file, you must also update the
 * `ROBOT_MAIN_CLASS` variable in the gradle build file. Note that
 * this file has a `@file:JvmName` annotation so that its compiled
 * Java class name is "Main" rather than "MainKt". This is to prevent
 * any issues/confusion if this file is ever replaced with a Java class.
 * See the [Package Level Functions](https://kotlinlang.org/docs/java-to-kotlin-interop.html#package-level-functions)
 * section on the *Calling Kotlin from Java* page of the Kotlin Docs.
 *
 * If you change your main Robot object (name), change the parameter of the
 * `RobotBase.startRobot` call below to the new name. (If you use the IDE's
 * Rename * Refactoring when renaming the object, it will get changed everywhere
 * including here.)
 */
fun main() = RobotBase.startRobot { CommandRobot }