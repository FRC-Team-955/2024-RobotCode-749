package frc.robot.subsystems.drivebase

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.Constants

class GyroIOReal : GyroIO() {
    private val pigeon = Pigeon2(Constants.Drivebase.pigeonId)
    private val yaw: StatusSignal<Double> = pigeon.yaw

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.setYaw(0.0)
        yaw.setUpdateFrequency(100.0)
        pigeon.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw) == StatusCode.OK
        inputs.yaw = Rotation2d.fromDegrees(yaw.refresh().valueAsDouble)
    }

    override fun setYaw(yaw: Rotation2d) {
        pigeon.setYaw(yaw.degrees)
    }
}
