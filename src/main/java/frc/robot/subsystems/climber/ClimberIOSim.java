package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim extends ClimberIO {
    private final DCMotorSim motor = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        motor.update(0.02);

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
}
