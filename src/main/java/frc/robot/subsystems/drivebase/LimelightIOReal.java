package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class LimelightIOReal extends LimelightIO {
    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        NetworkTable left = NetworkTableInstance.getDefault().getTable("limelight-left");
        inputs.leftTv = left.getEntry("tv").getInteger(-1);
        double[] leftBotpose = left.getEntry("botpose_wpiblue").getDoubleArray((double[]) null);
        if (leftBotpose != null && leftBotpose.length >= 11) {
            inputs.leftBotpose = new Pose2d(leftBotpose[0], leftBotpose[1], Rotation2d.fromDegrees(leftBotpose[5]));
            inputs.leftBotposeTimestamp = Timer.getFPGATimestamp() - (leftBotpose[6] / 1000.0);
            inputs.leftTagCount = leftBotpose[7];
            inputs.leftAvgArea = leftBotpose[10];
        } else {
            inputs.leftTv = -2;
            inputs.leftBotpose = noBotpose();
        }

        NetworkTable right = NetworkTableInstance.getDefault().getTable("limelight-right");
        inputs.rightTv = right.getEntry("tv").getInteger(-1);
        double[] rightBotpose = right.getEntry("botpose_wpiblue").getDoubleArray((double[]) null);
        if (rightBotpose != null && rightBotpose.length >= 11) {
            inputs.rightBotpose = new Pose2d(rightBotpose[0], rightBotpose[1], Rotation2d.fromDegrees(rightBotpose[5]));
            inputs.rightBotposeTimestamp = Timer.getFPGATimestamp() - (rightBotpose[6] / 1000.0);
            inputs.rightTagCount = rightBotpose[7];
            inputs.rightAvgArea = rightBotpose[10];
        } else {
            inputs.rightTv = -2;
            inputs.rightBotpose = noBotpose();
        }
    }
}
