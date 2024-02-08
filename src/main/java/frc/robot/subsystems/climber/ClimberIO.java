package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public class ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};

    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
  }

  public void updateInputs(ClimberIOInputs inputs) {
  }

  public void setRight(double speed) {
  }

  public void setLeft(double speed) {
  }

  public void stop() {
  }
}
