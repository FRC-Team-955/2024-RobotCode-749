package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim extends ClimberIO {
    private final DCMotorSim right = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);
    private final DCMotorSim left = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.leftAppliedVolts = left.getOutput(0);
        inputs.leftCurrentAmps = new double[]{left.getCurrentDrawAmps()};

        inputs.rightAppliedVolts = right.getOutput(0);
        inputs.rightCurrentAmps = new double[]{right.getCurrentDrawAmps()};
    }

    @Override
    public void setRight(double speed) {
        right.setInputVoltage(speed * 12.0);
    }

    @Override
    public void setLeft(double speed) {
        left.setInputVoltage(speed * 12.0);
    }

    @Override
    public void stop() {
        right.setInputVoltage(0);
        left.setInputVoltage(0);
    }
}
