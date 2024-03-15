package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

import static frc.robot.Util.ifSimElse;

public final class Constants {
    /**
     * Automatically determined based on if code is running on a real robot and if {@link Simulation#shouldReplay} is enabled
     */
    public static final Mode mode = RobotBase.isReal() ? Mode.REAL : (Simulation.shouldReplay ? Mode.REPLAY : Mode.SIM);

    public static final boolean useFileConstants = mode == Mode.SIM;

    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;

    public static final double controllerDeadzone = 0.05;
    public static final boolean useControllerDeadzone = true;

    public static final int pdhId = 12;

    public static final double errorRumbleAmount = 0.75;
    public static final double errorRumbleDuration = 0.25;

    public enum Mode {
        /**
         * Real robot
         */
        REAL,
        /**
         * Simulation
         */
        SIM,
        /**
         * Log replay
         */
        REPLAY,
    }

    public static final class Climber {
        public static final int rightMotorId = 7;
        public static final int leftMotorId = 8;

        public static final double gearRatio = 35;
        public static final double spoolDiameter = 0.75;
        public static final double ropeLength = 25.5;
        public static final double ropeLeftThreshold = 1;
    }

    public static final class Drivebase {
        public static final int leftLeaderMotorId = 1;
        public static final int leftFollowerMotorId = 2;
        public static final int rightLeaderMotorId = 4;
        public static final int rightFollowerMotorId = 3;
        public static final int pigeonId = 9;

        public static final DifferentialDrivetrainSim.KitbotWheelSize wheelSize = DifferentialDrivetrainSim.KitbotWheelSize.kSixInch;
        public static final double wheelRadius = wheelSize.value / 2; // value is diameter, we need radius
        public static final double trackWidth = Units.inchesToMeters(26);
        public static final DifferentialDrivetrainSim.KitbotGearing gearRatio = DifferentialDrivetrainSim.KitbotGearing.k10p71;
        public static final DifferentialDrivetrainSim.KitbotMotor motor = DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide;

        public static final int leftEncoderChannelA = 5;
        public static final int leftEncoderChannelB = 7;
        public static final int rightEncoderChannelA = 2;
        public static final int rightEncoderChannelB = 1;
        public static final double encoderDistancePerPulse = 1 / 2048.0; // AKA divide encoder reading by 2048

        public static final double velocityP = 1.2;
        public static final double velocityD = 0;

        public static final double feedforwardLeftS = ifSimElse(0.7, 1.0);
        public static final double feedforwardLeftV = ifSimElse(2.0, 2.2);
        public static final double feedforwardRightS = ifSimElse(feedforwardLeftS, 1.0);
        public static final double feedforwardRightV = ifSimElse(feedforwardLeftV, 2.2);

        public static final double swerveModeDeadzone = Simulation.useNintendoSwitchProController ? 0.5 : 0.8;
        public static final double swerveModeP = ifSimElse(0.08, 0.05);
        public static final double swerveModeD = ifSimElse(0.001, 0.008);

        public static final double pathfindMaxSpeed = 1.0;//2.5;
        public static final double pathfindMaxAccel = pathfindMaxSpeed;
        public static final double pathfindEndSpeed = 0.7;
    }

    public static class Intake {
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

    public static final class Launcher {
        public static final int topMotorId = 5;
        public static final int bottomMotorId = 6;

        public static final double launchingSpeed = 1;

        public static final double topIntakeSpeed = -1;
        public static final double bottomIntakeSpeed = -0.2;

        public static final double spinUpTime = 0.3;
    }

    public static final class Simulation {
        public static final boolean shouldReplay = false;
        public static final boolean useNintendoSwitchProController = Util.fileConstant("useNintendoSwitchProController", false);
    }
}
