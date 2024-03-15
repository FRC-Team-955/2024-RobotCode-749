package frc.robot.util

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

class TunablePIDController(private val name: String, kp: Double, ki: Double, kd: Double) : PIDController(kp, ki, kd),
    LoggableInputs {
    init {
        Shuffleboard.getTab("Tunable PID Controllers").add(name, this)
        process()
    }

    private fun process() {
        Logger.processInputs("Inputs/TunablePIDControllers/$name", this)
    }


    override fun toLog(table: LogTable) {
        table.put("P", p)
        table.put("I", i)
        table.put("D", d)
        table.put("IZone", iZone)
        table.put("PosTol", positionTolerance)
        table.put("VelTol", velocityTolerance)
    }

    override fun fromLog(table: LogTable) {
        System.out.printf(
            "TunablePIDControllers do not listen to log changes; values for %s are as follows:\n- P = %d\n- I = %d\n- D = %d\n- IZone = %d\n- PosTol = %d\n- VelTol = %d",
            name,
            table["P", -1],
            table["I", -1],
            table["D", -1],
            table["IZone", -1],
            table["PosTol", -1],
            table["VelTol", -1]
        )
    }

    override fun setP(kp: Double) {
        super.setP(kp)
        process()
    }

    override fun setI(ki: Double) {
        super.setI(ki)
        process()
    }

    override fun setD(kd: Double) {
        super.setD(kd)
        process()
    }

    override fun setIZone(iZone: Double) {
        super.setIZone(iZone)
        process()
    }

    override fun setSetpoint(setpoint: Double) {
        super.setSetpoint(setpoint)
        process()
    }

    override fun setTolerance(positionTolerance: Double) {
        super.setTolerance(positionTolerance)
        process()
    }

    override fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
        super.setTolerance(positionTolerance, velocityTolerance)
        process()
    }
}
