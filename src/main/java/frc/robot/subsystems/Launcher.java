package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    private final CANSparkMax feederMotor = new CANSparkMax(LauncherConstants.feederId, MotorType.kBrushed);
    private final CANSparkMax bottomMotor = new CANSparkMax(LauncherConstants.launcherId, MotorType.kBrushed);

    public Command startLaunchingCommand() {
        return this.runOnce(() -> {
            feederMotor.set(LauncherConstants.launchingSpeed);
            bottomMotor.set(LauncherConstants.launchingSpeed);
        });
    }

    public Command startIntakeCommand() {
        return this.runOnce(() -> {
            feederMotor.set(LauncherConstants.feederIntakeSpeed);
            bottomMotor.set(LauncherConstants.launcherIntakeSpeed);
        });
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            feederMotor.stopMotor();
            bottomMotor.stopMotor();
        });
    }
}
