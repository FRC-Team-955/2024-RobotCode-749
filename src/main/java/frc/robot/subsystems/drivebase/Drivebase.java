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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.GeneralConstants.Mode;
import frc.robot.subsystems.drivebase.commands.AutoAlign;
import frc.robot.subsystems.drivebase.commands.SwerveMode;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import static frc.robot.Util.chooseIO;

public class Drivebase extends SubsystemBase {
    private final DrivebaseIO io = chooseIO(DrivebaseIOReal::new, DrivebaseIOSim::new, DrivebaseIO::new);
    private final DrivebaseIOInputsAutoLogged inputs = new DrivebaseIOInputsAutoLogged();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrivebaseConstants.trackWidth);
    private final DifferentialDrivePoseEstimator odometry = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0.0, 0.0, new Pose2d());
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivebaseConstants.feedforwardS, DrivebaseConstants.feedforwardV);
    private final Field2d field = new Field2d();

    @AutoLogOutput
    private boolean arcadeDrive = false;
    @AutoLogOutput
    private boolean reverseMode = false;
    @AutoLogOutput
    private boolean preciseMode = false;

    /* Command Groups */
    public AutoAlign autoAlign = new AutoAlign(this);
    public SwerveMode swerveMode = new SwerveMode(this);

    public Drivebase() {
        SmartDashboard.putData("Field", field);

        AutoBuilder.configureRamsete(
                this::getPose,
                (pose) -> odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), pose),
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

        field.setRobotPose(odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters()));

        if (GeneralConstants.mode != Mode.SIM) {
            addPoseEstimation("limelight_left");
            addPoseEstimation("limelight_right");
        }
    }

    private void addPoseEstimation(String limeLightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limeLightName);
        if (table.getEntry("tv").getInteger(0) == 0)
            return;
        double[] botpose = table.getEntry("botpose").getDoubleArray((double[]) null);
        if (botpose == null)
            return;
        odometry.addVisionMeasurement(
                new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5])),
                Timer.getFPGATimestamp() - (botpose[6] / 1000.0));
    }

    public void arcadeDrive(double speed, double rotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(
                (preciseMode ? DrivebaseConstants.preciseModeMultiplier : 1) * (reverseMode ? -1 : 1) * (GeneralConstants.mode == GeneralConstants.Mode.REAL ? -rotation : speed),
                (preciseMode ? DrivebaseConstants.preciseModeMultiplier : 1) * (GeneralConstants.mode == GeneralConstants.Mode.REAL ? speed : rotation),
                true
        );
        io.setVoltage(speeds.left * 12, speeds.right * 12);
    }

    private void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
        Logger.recordOutput("Drivebase/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
        Logger.recordOutput("Drivebase/RightVelocitySetpointMetersPerSec", rightMetersPerSec);
        double leftRadPerSec = leftMetersPerSec / DrivebaseConstants.wheelRadius;
        double rightRadPerSec = rightMetersPerSec / DrivebaseConstants.wheelRadius;
        io.setVelocity(
                leftRadPerSec,
                rightRadPerSec,
                feedforward.calculate(leftRadPerSec),
                feedforward.calculate(rightRadPerSec)
        );
    }

    private Command arcadeDriveCommand(CommandXboxController controller) {
        return run(() -> arcadeDrive(controller.getLeftY(), -controller.getRightX()));
    }

    public Command pathfindCommand(Supplier<Pose2d> targetPoseSupplier) {
        return Commands.defer(() -> {
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
        }, Set.of());
    }

    public boolean getReverseMode() {
        return reverseMode;
    }

    public Command setReverseModeCommand(boolean reverseMode) {
        return Commands.runOnce(() -> this.reverseMode = reverseMode);
    }

    public boolean getPreciseMode() {
        return preciseMode;
    }

    public Command setPreciseModeCommand(boolean preciseMode) {
        return Commands.runOnce(() -> this.preciseMode = preciseMode);
    }

    public Command toggleArcadeDrive(CommandXboxController controller) {
        return Commands.runOnce(() -> {
            this.getCurrentCommand().cancel();
            if (arcadeDrive) {
                this.setDefaultCommand(swerveMode.swerveDriveCommand(controller));
            } else {
                this.setDefaultCommand(arcadeDriveCommand(controller));
            }
            arcadeDrive = !arcadeDrive;
        });
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public Command setPoseCommand(Pose2d newPose) {
        return Commands.runOnce(() -> odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), newPose));
    }

    public Command resetPoseCommand() {
        return Commands.runOnce(() -> odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d()));
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
