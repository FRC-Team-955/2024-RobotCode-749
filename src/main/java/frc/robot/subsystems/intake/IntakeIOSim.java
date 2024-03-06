package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSim extends IntakeIO {
    private final SingleJointedArmSim pivot = new SingleJointedArmSim(DCMotor.getNEO(1), IntakeConstants.pivotGearRatio, 0.0001, 0.3, -IntakeConstants.pivotRadDown, 0, true, 0);
    private final DCMotorSim driver = new DCMotorSim(DCMotor.getNEO(1), 1, 0.0001);

    private double pivotAppliedVolts = 0.0;
    private double driverAppliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        pivot.update(0.02);
        driver.update(0.02);

        inputs.pivotPositionRad = pivot.getAngleRads();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
        inputs.pivotCurrentAmps = pivot.getCurrentDrawAmps();

        inputs.driverAppliedVolts = driverAppliedVolts;
        inputs.driverCurrentAmps = driver.getCurrentDrawAmps();
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotAppliedVolts = volts;
        pivot.setInputVoltage(volts);
    }

    @Override
    public void resetPivotPosition() {
        pivot.setState(0, 0);
    }

    @Override
    public void setDriverVoltage(double volts) {
        driverAppliedVolts = volts;
        driver.setInputVoltage(volts);
    }

    @Override
    public void stopDriver() {
        driverAppliedVolts = 0;
        driver.setInputVoltage(0);
    }
}
