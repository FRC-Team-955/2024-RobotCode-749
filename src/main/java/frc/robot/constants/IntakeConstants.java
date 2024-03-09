package frc.robot.constants;

import edu.wpi.first.math.util.Units;

import static frc.robot.Util.ifSimElse;

public class IntakeConstants {
    public static final int pivotMotorId = 9;
    public static final int driverMotorId = 10;

    public static final double pivotGearRatio = 25;
    public static final double pivotRadDown = Units.degreesToRadians(195);
    public static final double pivotRadEject = pivotRadDown - Units.degreesToRadians(80);
    public static final double pivotP = 2.5;
    public static final double pivotD = 0;
    public static final double pivotFFg = ifSimElse(0.01, 0.35);

    public static final double intakeSpeed = 1;
    public static final double handoffSpeed = -0.75;
    public static final double handoffTimeout = 0.15;
    public static final double ejectSpeed = -1;
    public static final double ejectTimeout = 0.5;
    public static final double ejectIntakeTimeout = 0.15;
}
