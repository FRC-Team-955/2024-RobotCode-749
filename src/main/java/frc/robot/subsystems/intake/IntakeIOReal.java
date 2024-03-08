package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.IntakeConstants;

public class IntakeIOReal extends IntakeIO {
    private final CANSparkMax pivot = new CANSparkMax(IntakeConstants.pivotMotorId, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax driver = new CANSparkMax(IntakeConstants.driverMotorId, CANSparkLowLevel.MotorType.kBrushless);

    private final RelativeEncoder pivotEncoder = pivot.getEncoder();

    public IntakeIOReal() {
        pivot.restoreFactoryDefaults();
        driver.restoreFactoryDefaults();

        pivot.setIdleMode(CANSparkBase.IdleMode.kBrake);
        driver.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pivot.setCANTimeout(250);
        driver.setCANTimeout(250);

        pivot.enableVoltageCompensation(12.0);
        driver.enableVoltageCompensation(12.0);
        pivot.setSmartCurrentLimit(40);
        driver.setSmartCurrentLimit(40);

        pivot.burnFlash();
        driver.burnFlash();

        pivotEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotPositionRad = Units.rotationsToRadians(pivotEncoder.getPosition() / IntakeConstants.pivotGearRatio);
        inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity());
        inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
        inputs.pivotCurrentAmps = pivot.getOutputCurrent();

        inputs.driverAppliedVolts = driver.getAppliedOutput() * driver.getBusVoltage();
        inputs.driverCurrentAmps = driver.getOutputCurrent();
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivot.setVoltage(volts);
    }

    @Override
    public void resetPivotPosition() {
        pivotEncoder.setPosition(0);
    }

    @Override
    public void setDriverVoltage(double volts) {
        driver.setVoltage(volts);
    }

    @Override
    public void stopDriver() {
        driver.stopMotor();
    }
}
