package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public final class DrivebaseConstants {
    public final static int leftLeaderMotorId = 1;
    public final static int leftFollowerMotorId = 2;
    public final static int rightLeaderMotorId = 3;
    public final static int rightFollowerMotorId = 4;

    public static final KitbotWheelSize wheelSize = KitbotWheelSize.kSixInch;
    public static final double wheelRadius = wheelSize.value / 2; // value is diameter, we need radius
    public static final double trackWidth = Units.inchesToMeters(26);
    public static final KitbotGearing gearRatio = KitbotGearing.k10p71;
    public static final KitbotMotor motor = KitbotMotor.kDualCIMPerSide;

    public static final double motorP = 1;
    public static final double motorD = 0;

    public static final double feedforwardS = 0;
    public static final double feedforwardV = GeneralConstants.mode == GeneralConstants.Mode.SIM ? 0.35 : 0;

    public static final double preciseModeMultiplier = 0.75;

    public static final double swerveModeDeadzone = 0.25;
    public static final double swerveModeP = GeneralConstants.mode == GeneralConstants.Mode.SIM ? 0.03 : 0;
    public static final double swerveModeD = GeneralConstants.mode == GeneralConstants.Mode.SIM ? 0.0001 : 0;
}
