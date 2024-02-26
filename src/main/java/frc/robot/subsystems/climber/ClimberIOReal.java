package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

public abstract class ClimberIOReal extends ClimberIO {
    private final CANSparkMax motor = getMotor();

    protected abstract CANSparkMax getMotor();

    public ClimberIOReal() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setCANTimeout(250);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(40);
        motor.burnFlash();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void set(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
