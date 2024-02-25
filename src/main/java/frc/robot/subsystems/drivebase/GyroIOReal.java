package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DrivebaseConstants;

public class GyroIOReal extends GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(DrivebaseConstants.pigeonId);
    private final StatusSignal<Double> yaw = pigeon.getYaw();

    public GyroIOReal() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.setYaw(0);
        yaw.setUpdateFrequency(100);
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble());
    }

    @Override
    public void resetYaw() {
        pigeon.setYaw(0);
    }
}
