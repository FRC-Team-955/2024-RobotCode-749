package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {
    private final CANSparkMax leftMotorLeader = new CANSparkMax(DrivebaseConstants.leftMotorLeader, MotorType.kBrushed);
    private final CANSparkMax leftMotorFollower = new CANSparkMax(DrivebaseConstants.leftMotorFollower, MotorType.kBrushed);
    private final CANSparkMax rightMotorLeader = new CANSparkMax(DrivebaseConstants.rightMotorLeader, MotorType.kBrushed);
    private final CANSparkMax rightMotorFollower = new CANSparkMax(DrivebaseConstants.rightMotorFollower, MotorType.kBrushed);

    public Drivebase() {
        leftMotorFollower.follow(leftMotorLeader);
        rightMotorFollower.follow(rightMotorLeader);
    }

    private final DifferentialDrive drive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(rotation, speed);
    }
}
