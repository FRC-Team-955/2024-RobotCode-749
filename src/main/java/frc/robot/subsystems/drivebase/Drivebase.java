package frc.robot.subsystems.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.subsystems.drivebase.commands.AutoAlign;
import frc.robot.subsystems.drivebase.commands.SwerveMode;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.Util.chooseIO;

public class Drivebase extends SubsystemBase {
    private final DrivebaseIO io = chooseIO(DrivebaseIOReal::new, DrivebaseIOSim::new, DrivebaseIO::new);
    private final DrivebaseIOInputsAutoLogged inputs = new DrivebaseIOInputsAutoLogged();

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrivebaseConstants.trackWidth);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivebaseConstants.feedforwardS, DrivebaseConstants.feedforwardV);

    /* Command Groups */
    public AutoAlign autoAlign = new AutoAlign(this);
    public SwerveMode swerveMode = new SwerveMode(this);

    public Drivebase() {
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

        odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters());
    }

    public void arcadeDrive(double speed, double rotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(
                GeneralConstants.mode == GeneralConstants.Mode.REAL ? rotation : speed,
                GeneralConstants.mode == GeneralConstants.Mode.REAL ? speed : rotation,
                true
        );
        io.setVoltage(speeds.left * 12, speeds.right * 12);
    }

    public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
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

    public Command arcadeDriveCommand(CommandXboxController controller, boolean preciseMode) {
        if (preciseMode) {
            return run(() -> arcadeDrive(controller.getLeftY() * DrivebaseConstants.preciseModeMultiplier, -controller.getRightX() * DrivebaseConstants.preciseModeMultiplier));
        } else {
            return run(() -> arcadeDrive(controller.getLeftY(), -controller.getRightX()));
        }
    }

    public Command pathfindCommand(Supplier<Pose2d> targetPoseSupplier) {
        return Commands.runOnce(() -> {
            var pose = getPose();
            var targetPose = targetPoseSupplier.get();

            // Check if the robot is already at the target pose.
            if (Math.abs(pose.getX() - targetPose.getX()) <= .1 && Math.abs(pose.getY() - targetPose.getY()) <= .1) {
                return;
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
            AutoBuilder
                    .followPath(path)
                    .andThen(swerveMode.swerveAngleCommand(targetPose.getRotation().getDegrees()))
                    .schedule();
        });
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
