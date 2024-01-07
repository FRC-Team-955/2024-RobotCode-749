package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
    private CANSparkMax motorLeft1 = new CANSparkMax(1, MotorType.kBrushed);
    private CANSparkMax motorLeft2 = new CANSparkMax(2, MotorType.kBrushed);
    private CANSparkMax motorRight1 = new CANSparkMax(3, MotorType.kBrushed);
    private CANSparkMax motorRight2 = new CANSparkMax(4, MotorType.kBrushed);

    private MotorControllerGroup leftMotors = new MotorControllerGroup(motorLeft1, motorLeft2);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(motorRight1, motorRight2);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    public void arcadeDrive(double speed, double rotation){
        drive.arcadeDrive(rotation, speed);
    }
}
