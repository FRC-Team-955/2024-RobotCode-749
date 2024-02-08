package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.constants.ClimberConstants;

public class ClimberIOReal extends ClimberIO {
  private final WPI_TalonSRX right = new WPI_TalonSRX(ClimberConstants.rightMotorId);
  private final WPI_TalonSRX left = new WPI_TalonSRX(ClimberConstants.leftMotorId);

  public ClimberIOReal() {
    var config = new TalonSRXConfiguration();
    config.peakCurrentLimit = 80;
    config.peakCurrentDuration = 250;
    config.continuousCurrentLimit = 60;
    config.voltageCompSaturation = 12.0;
    right.configAllSettings(config);
    left.configAllSettings(config);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftAppliedVolts = left.getMotorOutputVoltage();
    inputs.leftCurrentAmps = new double[] { left.getStatorCurrent() };

    inputs.rightAppliedVolts = right.getMotorOutputVoltage();
    inputs.rightCurrentAmps = new double[] { right.getStatorCurrent() };
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
