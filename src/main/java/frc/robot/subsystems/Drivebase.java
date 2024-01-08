package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {
    private CANSparkMax leftMotor1 = new CANSparkMax(DrivebaseConstants.leftMotorId1, MotorType.kBrushed);
    private CANSparkMax leftMotor2 = new CANSparkMax(DrivebaseConstants.leftMotorId2, MotorType.kBrushed);
    private CANSparkMax rightMotor1 = new CANSparkMax(DrivebaseConstants.rightMotorId1, MotorType.kBrushed);
    private CANSparkMax rightMotor2 = new CANSparkMax(DrivebaseConstants.rightMotorId2, MotorType.kBrushed);

    private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    public void arcadeDrive(double speed, double rotation){
        drive.arcadeDrive(rotation, speed);
    }
}
