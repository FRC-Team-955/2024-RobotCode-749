package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

import static frc.robot.Util.ifSimElse;

public final class DrivebaseConstants {
    public static final int leftLeaderMotorId = 1;
    public static final int leftFollowerMotorId = 2;
    public static final int rightLeaderMotorId = 4;
    public static final int rightFollowerMotorId = 3;
    public static final int pigeonId = 9;

    public static final KitbotWheelSize wheelSize = KitbotWheelSize.kSixInch;
    public static final double wheelRadius = wheelSize.value / 2; // value is diameter, we need radius
    public static final double trackWidth = Units.inchesToMeters(26);
    public static final KitbotGearing gearRatio = KitbotGearing.k10p71;
    public static final KitbotMotor motor = KitbotMotor.kDualCIMPerSide;

    public static final int leftEncoderChannelA = 5;
    public static final int leftEncoderChannelB = 7;
    public static final int rightEncoderChannelA = 2;
    public static final int rightEncoderChannelB = 1;
    public static final double encoderDistancePerPulse = 1 / 2048.0; // AKA divide encoder reading by 2048

    public static final double velocityP = ifSimElse(0.05, .1);
    public static final double velocityD = 0.001;

    public static final double feedforwardLeftS = ifSimElse(0.0, 0.90132);
    public static final double feedforwardLeftV = ifSimElse(0.35, 2.67411);
    public static final double feedforwardRightS = ifSimElse(feedforwardLeftS, 1.10306);
    public static final double feedforwardRightV = ifSimElse(feedforwardLeftV, 2.77553);

    public static final double preciseModeMultiplier = 0.6;

    public static final double swerveModeDeadzone = 0.25;
    public static final double swerveModeP = ifSimElse(0.03, 0.018);
    public static final double swerveModeD = 0.0001;
}
