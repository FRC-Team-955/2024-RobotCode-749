package frc.robot.subsystems.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.Util.ifRealElse;
import static frc.robot.Util.switchMode;

public class Drivebase extends SubsystemBase {
    private final DrivebaseIO io = switchMode(DrivebaseIOReal::new, DrivebaseIOSim::new, DrivebaseIO::new);
    private final DrivebaseIOInputsAutoLogged inputs = new DrivebaseIOInputsAutoLogged();

    private final GyroIO gyroIO = switchMode(GyroIOReal::new, GyroIOSim::new, GyroIO::new);
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final LimelightIO limelightIO = ifRealElse(LimelightIOReal::new, LimelightIO::new);
    private final LimelightIOInputsAutoLogged limelightInputs = new LimelightIOInputsAutoLogged();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrivebaseConstants.trackWidth);
    private final DifferentialDrivePoseEstimator odometry = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0.0, 0.0, new Pose2d());
    private final SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(DrivebaseConstants.feedforwardLeftS, DrivebaseConstants.feedforwardLeftV);
    private final SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(DrivebaseConstants.feedforwardRightS, DrivebaseConstants.feedforwardRightV);
    private final TunablePIDController driveVelocityPID = new TunablePIDController("Drivebase driveVelocity", DrivebaseConstants.velocityP, 0, DrivebaseConstants.velocityD);
    private final Field2d field = new Field2d();

    @AutoLogOutput(key = "Drivebase/ArcadeDrive/Enabled")
    private boolean arcadeDrive = false;
    @AutoLogOutput
    private boolean reverseMode = false;
    @AutoLogOutput
    private boolean preciseMode = false;

    /* Command Groups */
    public final AutoAlign autoAlign = new AutoAlign(this);
    public final SwerveMode swerveMode = new SwerveMode(this);

    public Drivebase() {
        SmartDashboard.putData("Field", field);

        AutoBuilder.configureRamsete(
                this::getPose,
                (pose) -> odometry.resetPosition(gyroInputs.yaw, getLeftPositionMeters(), getRightPositionMeters(), pose),
                () -> kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
                (speeds) -> {
                    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
                    driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
                },
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

        if (limelightInputs.leftTv == 1)
            odometry.addVisionMeasurement(limelightInputs.leftBotpose, limelightInputs.leftBotposeTimestamp);
        if (limelightInputs.rightTv == 1)
            odometry.addVisionMeasurement(limelightInputs.rightBotpose, limelightInputs.rightBotposeTimestamp);
    }

    /**
     * @param speed    Positive = forward
     * @param rotation Positive = counterclockwise
     */
    public void arcadeDrive(double speed, double rotation) {
        Logger.recordOutput("Drivebase/ArcadeDrive/Speed", speed);
        Logger.recordOutput("Drivebase/ArcadeDrive/Rotation", rotation);
        var speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, true);
        io.setVoltage(speeds.left * 12, speeds.right * 12);
    }

    private void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
        Logger.recordOutput("Drivebase/DriveVelocity/LeftSetpointMetersPerSec", leftMetersPerSec);
        Logger.recordOutput("Drivebase/DriveVelocity/RightSetpointMetersPerSec", rightMetersPerSec);
        var leftPID = driveVelocityPID.calculate(getLeftVelocityMetersPerSec(), leftMetersPerSec);
        var rightPID = driveVelocityPID.calculate(getRightVelocityMetersPerSec(), rightMetersPerSec);
        var leftFF = leftFeedforward.calculate(leftMetersPerSec);
        var rightFF = rightFeedforward.calculate(rightMetersPerSec);
        Logger.recordOutput("Drivebase/DriveVelocity/LeftControlSignalPID", leftPID);
        Logger.recordOutput("Drivebase/DriveVelocity/RightControlSignalPID", rightPID);
        Logger.recordOutput("Drivebase/DriveVelocity/LeftControlSignalFF", leftFF);
        Logger.recordOutput("Drivebase/DriveVelocity/RightControlSignalFF", rightFF);
        io.setVoltage(leftPID + leftFF, rightPID + rightFF);
    }

    public Command driveVelocityCommand(double leftMetersPerSec, double rightMetersPerSec) {
        return this.run(() -> driveVelocity(leftMetersPerSec, rightMetersPerSec));
    }

    private Command arcadeDriveCommand(CommandXboxController controller) {
        return run(() -> {
            var precise = preciseMode ? DrivebaseConstants.preciseModeMultiplier : 1;
            var reverse = reverseMode ? -1 : 1;

            var speed = precise * reverse * -controller.getLeftY();
            var rotation = precise * -controller.getRightX();

            if (GeneralConstants.useControllerDeadzone) {
                if (Math.abs(speed) < GeneralConstants.controllerDeadzone) speed = 0;
                if (Math.abs(rotation) < GeneralConstants.controllerDeadzone) rotation = 0;
            }

            arcadeDrive(speed, rotation);
        });
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
                    new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                    new GoalEndState(0.0, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            path.preventFlipping = true;
            return AutoBuilder
                    .followPath(path)
                    .andThen(swerveMode.swerveAngleCommand(targetPose.getRotation().getDegrees()));
        });
    }

    public boolean getReverseMode() {
        return reverseMode;
    }

    public Command enableReverseModeCommand() {
        return Commands.startEnd(
                () -> this.reverseMode = true,
                () -> this.reverseMode = false
        );
    }

    public boolean getPreciseMode() {
        return preciseMode;
    }

    public Command enablePreciseModeCommand() {
        return Commands.startEnd(
                () -> this.preciseMode = true,
                () -> this.preciseMode = false
        );
    }

    public Command toggleArcadeDrive(CommandXboxController controller) {
        return Commands.runOnce(() -> {
            if (this.getCurrentCommand() == this.getDefaultCommand())
                this.getCurrentCommand().cancel();
            arcadeDrive = !arcadeDrive;
            if (arcadeDrive) {
                this.setDefaultCommand(arcadeDriveCommand(controller));
            } else {
                this.setDefaultCommand(swerveMode.swerveDriveCommand(controller));
            }
        });
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
        return Commands.runOnce(() -> odometry.resetPosition(gyroInputs.yaw, getLeftPositionMeters(), getRightPositionMeters(), newPose));
    }

    public Command resetGyroCommand() {
        return runOnce(gyroIO::resetYaw).andThen(swerveMode.swerveAngleCommand(0));
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
