package frc.robot.subsystems.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util;
import frc.robot.commands.FeedforwardCharacterization;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.subsystems.drivebase.commands.AutoAlign;
import frc.robot.subsystems.drivebase.commands.SwerveMode;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.Util.ifRealElse;
import static frc.robot.Util.switchMode;

public class Drivebase extends SubsystemBase {
    private final CommandXboxController driverController;

    private final DrivebaseIO io = switchMode(DrivebaseIOReal::new, DrivebaseIOSim::new, DrivebaseIO::new);
    private final DrivebaseIOInputsAutoLogged inputs = new DrivebaseIOInputsAutoLogged();

    private final GyroIO gyroIO = switchMode(GyroIOReal::new, GyroIOSim::new, GyroIO::new);
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final LimelightIO limelightIO = ifRealElse(LimelightIOReal::new, LimelightIO::new);
    private final LimelightIOInputsAutoLogged limelightInputs = new LimelightIOInputsAutoLogged();
    private final LoggedDashboardBoolean usePoseEstimation = new LoggedDashboardBoolean("Use Pose Estimation", true);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrivebaseConstants.trackWidth);
    private final DifferentialDrivePoseEstimator odometry = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0.0, 0.0, new Pose2d());
    private final Field2d field = new Field2d();
    private final SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(DrivebaseConstants.feedforwardLeftS, DrivebaseConstants.feedforwardLeftV);
    private final SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(DrivebaseConstants.feedforwardRightS, DrivebaseConstants.feedforwardRightV);
    private final TunablePIDController driveVelocityPID = new TunablePIDController("Drivebase driveVelocity", DrivebaseConstants.velocityP, 0, DrivebaseConstants.velocityD);
    private final LoggedDashboardBoolean disableDriving = new LoggedDashboardBoolean("Disable Driving", false);

    private final LoggedDashboardBoolean arcadeDriveToggle = new LoggedDashboardBoolean("Arcade Drive", false);
    private boolean arcadeDrive = arcadeDriveToggle.get();
    @AutoLogOutput
    private boolean reverseMode = false;

    /* Command Groups */
    public final AutoAlign autoAlign = new AutoAlign(this);
    public final SwerveMode swerveMode;

    public Drivebase(CommandXboxController driverController) {
        this.driverController = driverController;
        swerveMode = new SwerveMode(this.driverController, this);

        SmartDashboard.putData("Field", field);

        AutoBuilder.configureLTV(
                this::getPose,
                (pose) -> odometry.resetPosition(gyroInputs.yaw, getLeftPositionMeters(), getRightPositionMeters(), pose),
                () -> kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
                (speeds) -> {
                    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
                    driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
                },
                0.02,
                new ReplanningConfig(),
                Util::shouldFlip,
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput("Drivebase/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput("Drivebase/TrajectorySetpoint", targetPose));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Drivebase", inputs);

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Inputs/Gyro", gyroInputs);

        limelightIO.updateInputs(limelightInputs);
        Logger.processInputs("Inputs/Limelight", limelightInputs);

        field.setRobotPose(odometry.update(gyroInputs.yaw, getLeftPositionMeters(), getRightPositionMeters()));

        if (usePoseEstimation.get()) {
            if (limelightInputs.leftTv == 1)
                addVisionMeasurement(limelightInputs.leftBotpose, limelightInputs.leftBotposeTimestamp, limelightInputs.leftTagCount, limelightInputs.leftAvgArea);
            if (limelightInputs.rightTv == 1)
                addVisionMeasurement(limelightInputs.rightBotpose, limelightInputs.rightBotposeTimestamp, limelightInputs.leftTagCount, limelightInputs.leftAvgArea);
        } else {
            // Revert rotation changes
            odometry.addVisionMeasurement(
                    new Pose2d(getPose().getX(), getPose().getY(), gyroInputs.yaw),
                    Timer.getFPGATimestamp(),
                    VecBuilder.fill(0, 0, 0)
            );
        }

        if (arcadeDriveToggle.get() != arcadeDrive) {
            arcadeDrive = arcadeDriveToggle.get();
            updateDefaultCommand();
        }
    }

    public Command teleopInitCommand() {
        return runOnce(() -> {
            usePoseEstimation.set(false);
            gyroIO.setYaw(getPose().getRotation().getDegrees());
            odometry.addVisionMeasurement(getPose(), Timer.getFPGATimestamp(), VecBuilder.fill(0, 0, 0));
        });
    }

    private void addVisionMeasurement(Pose2d botpose, double timestamp, double tagCount, double avgArea) {
        double odometryDifference = odometry.getEstimatedPosition().getTranslation().getDistance(botpose.getTranslation());

        double xyStdDev;
        double rotStdDev;

        if (avgArea > 0.8 && odometryDifference < 0.5) {
            xyStdDev = 1;
            rotStdDev = 10;
        } else if (avgArea > 0.8) {
            xyStdDev = 1.5;
            rotStdDev = 10;
        } else if (avgArea > 0.5 && odometryDifference < 1) {
            xyStdDev = 2;
            rotStdDev = 15;
        } else if (avgArea > 0.2 && odometryDifference < 2) {
            xyStdDev = 4;
            rotStdDev = 30;
        } else if (avgArea > 0.05 && odometryDifference < 5) {
            xyStdDev = 10;
            rotStdDev = 30;
        } else return;

        if (tagCount >= 2) {
            xyStdDev -= avgArea > 0.8 ? 0.25 : 0.5;
            rotStdDev -= 8;
        }

        odometry.addVisionMeasurement(
                botpose,
                timestamp,
                VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(rotStdDev))
        );
    }

    /**
     * @param speed    Positive = forward
     * @param rotation Positive = counterclockwise
     */
    public void arcadeDrive(double speed, double rotation) {
        Logger.recordOutput("Drivebase/ArcadeDrive/Speed", speed);
        Logger.recordOutput("Drivebase/ArcadeDrive/Rotation", rotation);
        var speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, true);
        if (!disableDriving.get()) io.setVoltage(speeds.left * 12, speeds.right * 12);
    }

    private void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
        Logger.recordOutput("Drivebase/DriveVelocity/LeftSetpointMetersPerSec", leftMetersPerSec);
        Logger.recordOutput("Drivebase/DriveVelocity/RightSetpointMetersPerSec", rightMetersPerSec);

        var leftPID = driveVelocityPID.calculate(getLeftVelocityMetersPerSec(), leftMetersPerSec);
        var rightPID = driveVelocityPID.calculate(getRightVelocityMetersPerSec(), rightMetersPerSec);
        Logger.recordOutput("Drivebase/DriveVelocity/LeftControlSignalPID", leftPID);
        Logger.recordOutput("Drivebase/DriveVelocity/RightControlSignalPID", rightPID);

        var leftFF = leftFeedforward.calculate(leftMetersPerSec);
        var rightFF = rightFeedforward.calculate(rightMetersPerSec);
        Logger.recordOutput("Drivebase/DriveVelocity/LeftControlSignalFF", leftFF);
        Logger.recordOutput("Drivebase/DriveVelocity/RightControlSignalFF", rightFF);

        io.setVoltage(leftPID + leftFF, rightPID + rightFF);
    }

    public Command driveVelocityCommand(double leftMetersPerSec, double rightMetersPerSec) {
        return this.run(() -> driveVelocity(leftMetersPerSec, rightMetersPerSec)).withName("Drivebase$driveVelocity");
    }

    private Command arcadeDriveCommand() {
        return run(() -> {
            var reverse = reverseMode ? -1 : 1;

            var speed = reverse * Util.speed(driverController);
            var rotation = -driverController.getLeftX();

            if (GeneralConstants.useControllerDeadzone) {
                if (Math.abs(speed) < GeneralConstants.controllerDeadzone) speed = 0;
                if (Math.abs(rotation) < GeneralConstants.controllerDeadzone) rotation = 0;
            }

            arcadeDrive(speed, rotation);
        }).withName("Drivebase$arcadeDrive");
    }

    public Command followPathCommand(String name) {
        var path = PathPlannerPath.fromPathFile(name);
        return setPoseCommand(Util.flipIfNeeded(path.getStartingDifferentialPose())).andThen(AutoBuilder.followPath(path)).withName("Drivebase$followPath");
    }

    public Command pathfindCommand(Supplier<Pose2d> targetPoseSupplier) {
        return Commands.deferredProxy(() -> {
            var pose = getPose();
            var targetPose = targetPoseSupplier.get();

            // Check if the robot is already at the target pose.
            if (Math.abs(pose.getX() - targetPose.getX()) <= .1 && Math.abs(pose.getY() - targetPose.getY()) <= .1) {
                return Commands.none();
            }

            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    pose,
                    targetPose
            );

            PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(DrivebaseConstants.pathfindMaxSpeed, DrivebaseConstants.pathfindMaxAccel, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                    new GoalEndState(DrivebaseConstants.pathfindEndSpeed, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            path.preventFlipping = true;
            return AutoBuilder
                    .followPath(path)
                    .andThen(swerveMode.swerveAngleCommand(targetPose.getRotation().getDegrees()));
        }).withName("Drivebase$pathfind");
    }

    public boolean getReverseMode() {
        return reverseMode;
    }

    public Command toggleReverseModeCommand() {
        return Commands.runOnce(() -> reverseMode = !reverseMode);
    }

    public Command toggleArcadeDrive() {
        return Commands.runOnce(() -> {
            var newVal = !arcadeDriveToggle.get();
            arcadeDriveToggle.set(newVal);
            arcadeDrive = newVal;
            updateDefaultCommand();
        });
    }

    public void updateDefaultCommand() {
        System.out.println("Updating default command (arcadeDrive = " + arcadeDrive + ")");
        if (this.getCurrentCommand() == this.getDefaultCommand())
            this.getCurrentCommand().cancel();
        if (arcadeDrive) {
            this.setDefaultCommand(arcadeDriveCommand());
        } else {
            this.setDefaultCommand(swerveMode.swerveDriveCommand());
        }
    }

    public Command feedforwardCharacterizationLeft() {
        return new FeedforwardCharacterization(this, (volts) -> io.setVoltage(volts, volts), this::getLeftVelocityMetersPerSec);
    }

    public Command feedforwardCharacterizationRight() {
        return new FeedforwardCharacterization(this, (volts) -> io.setVoltage(volts, volts), this::getRightVelocityMetersPerSec);
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public Command setPoseCommand(Pose2d newPose) {
        return setPoseCommand(() -> newPose);
    }

    public Command setPoseCommand(Supplier<Pose2d> newPose) {
        return Commands.runOnce(() -> odometry.resetPosition(gyroInputs.yaw, getLeftPositionMeters(), getRightPositionMeters(), newPose.get()));
    }

    public Rotation2d getGyro() {
        return gyroInputs.yaw;
    }

    public Command resetGyroCommand() {
        return runOnce(() -> gyroIO.setYaw(0)).andThen(swerveMode.swerveAngleCommand(0));
    }

    @AutoLogOutput
    public double getLeftPositionMeters() {
        return inputs.leftPositionRad * DrivebaseConstants.wheelRadius;
    }

    @AutoLogOutput
    public double getRightPositionMeters() {
        return inputs.rightPositionRad * DrivebaseConstants.wheelRadius;
    }

    @AutoLogOutput
    public double getLeftVelocityMetersPerSec() {
        return inputs.leftVelocityRadPerSec * DrivebaseConstants.wheelRadius;
    }

    @AutoLogOutput
    public double getRightVelocityMetersPerSec() {
        return inputs.rightVelocityRadPerSec * DrivebaseConstants.wheelRadius;
    }
}
