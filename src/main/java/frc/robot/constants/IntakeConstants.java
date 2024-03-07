package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final int pivotMotorId = 9;
    public static final int driverMotorId = 10;

    public static final double pivotGearRatio = 25;
    public static final double pivotRadDown = Units.degreesToRadians(185);

    public static final double intakeSpeed = 1;
    public static final double handoffSpeed = -0.75;
    public static final double handoffTimeout = 0.2;
}
