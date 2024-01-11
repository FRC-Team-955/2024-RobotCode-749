package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    private final CANSparkMax feederMotor = new CANSparkMax(LauncherConstants.feederId, MotorType.kBrushed);
    private final CANSparkMax bottomMotor = new CANSparkMax(LauncherConstants.launcherId, MotorType.kBrushed);

    private void startLaunching() {
        feederMotor.set(LauncherConstants.launchingSpeed);
        bottomMotor.set(LauncherConstants.launchingSpeed);
    }

    private void startIntake() {
        feederMotor.set(LauncherConstants.feederIntakeSpeed);
        bottomMotor.set(LauncherConstants.launcherIntakeSpeed);
    }

    private void stopMotors() {
        feederMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    public Command launchCommand() {
        return this.startEnd(
                this::startLaunching,
                this::stopMotors
        );
    }

    public Command intakeCommand() {
        return this.startEnd(
                this::startIntake,
                this::stopMotors
        );
    }
}
