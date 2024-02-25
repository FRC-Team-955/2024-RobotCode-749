package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.constants.ClimberConstants;

public class ClimberIOReal extends ClimberIO {
    private final CANSparkMax left = new CANSparkMax(ClimberConstants.leftMotorId, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax right = new CANSparkMax(ClimberConstants.rightMotorId, CANSparkLowLevel.MotorType.kBrushless);

    public ClimberIOReal() {
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        left.setIdleMode(CANSparkBase.IdleMode.kBrake);
        right.setIdleMode(CANSparkBase.IdleMode.kBrake);

        left.setCANTimeout(250);
        right.setCANTimeout(250);

        left.enableVoltageCompensation(12.0);
        right.enableVoltageCompensation(12.0);
        left.setSmartCurrentLimit(40);
        right.setSmartCurrentLimit(40);

        left.burnFlash();
        right.burnFlash();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.leftAppliedVolts = left.getAppliedOutput() * left.getBusVoltage();
        inputs.leftCurrentAmps = left.getOutputCurrent();

        inputs.rightAppliedVolts = right.getAppliedOutput() * right.getBusVoltage();
        inputs.rightCurrentAmps = right.getOutputCurrent();
    }

    @Override
    public void setRight(double speed) {
        right.setVoltage(speed * 12.0);
    }

    @Override
    public void setLeft(double speed) {
        left.setVoltage(speed * 12.0);
    }

    @Override
    public void stop() {
        right.stopMotor();
        left.stopMotor();
    }
}
