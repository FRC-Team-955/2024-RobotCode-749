package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ClimberConstants;

public class ClimberIOSim extends ClimberIO {
    private final DCMotorSim motor = new DCMotorSim(DCMotor.getNEO(1), ClimberConstants.gearRatio, 0.0001);
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        motor.update(0.02);

        inputs.positionRad = motor.getAngularPositionRad();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = motor.getCurrentDrawAmps();
    }

    @Override
    public void set(double volts) {
        appliedVolts = volts;
        motor.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        appliedVolts = 0;
        motor.setInputVoltage(0);
    }

    @Override
    public void resetPosition() {
        motor.setState(0, 0);
    }
}
