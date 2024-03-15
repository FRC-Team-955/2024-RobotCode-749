package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public abstract class ClimberIOReal extends ClimberIO {
    private final CANSparkMax motor = getMotor();
    private final RelativeEncoder encoder = motor.getEncoder();

    protected abstract CANSparkMax getMotor();

    public ClimberIOReal() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setCANTimeout(250);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(40);
        motor.burnFlash();
        encoder.setPosition(0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / Constants.Climber.gearRatio);
    }

    @Override
    public void set(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void resetPosition() {
        encoder.setPosition(0);
    }
}
