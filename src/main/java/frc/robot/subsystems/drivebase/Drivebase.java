package frc.robot.subsystems.drivebase;

import static frc.robot.Util.chooseIO;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TunablePIDController;

public class Drivebase extends SubsystemBase {
    private final DrivebaseIO io = chooseIO(DrivebaseIOReal::new, DrivebaseIOSim::new, DrivebaseIO::new);
    private final DrivebaseIOInputsAutoLogged inputs = new DrivebaseIOInputsAutoLogged();

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrivebaseConstants.trackWidth);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivebaseConstants.feedforwardS, DrivebaseConstants.feedforwardV);

    private final TunablePIDController swerveModePID = Util.make(() -> {
        var pid = new TunablePIDController("Swerve Mode", DrivebaseConstants.swerveModeP, 0, DrivebaseConstants.swerveModeD);
        pid.enableContinuousInput(-180, 180);
        return pid;
    });
    private double swerveModeSetpoint = 0;

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

    private void arcadeDrive(double speed, double rotation) {
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

    public Command swerveDriveCommand(CommandXboxController controller, boolean preciseMode) {
        return Commands
                .runOnce(() -> swerveModeSetpoint = getPose().getRotation().getDegrees())
                .andThen(run(() -> {
                    var x = controller.getRightX();
                    var y = controller.getRightY();

                    if (Math.abs(x) > DrivebaseConstants.swerveModeDeadzone || Math.abs(y) > DrivebaseConstants.swerveModeDeadzone) {
                        swerveModeSetpoint = Math.toDegrees(Math.atan2(-x, y));
                    }

                    swerveModePID.setSetpoint(swerveModeSetpoint);
                    Logger.recordOutput("Drivebase/SwerveMode/Setpoint", swerveModeSetpoint);
                    var robotAngle = getPose().getRotation().getDegrees();
                    var rotation = swerveModePID.calculate(robotAngle);
                    Logger.recordOutput("Drivebase/SwerveMode/Rotation", rotation);
                    if (preciseMode) {
                        arcadeDrive(controller.getLeftY() * DrivebaseConstants.preciseModeMultiplier, rotation);
                    } else {
                        arcadeDrive(controller.getLeftY(), rotation);
                    }
                }));
    }

    public Command swerveAngleCommand(double angle) {
        return Commands.runOnce(() -> swerveModeSetpoint = angle);
    }

    public Command followPathCommand(String pathName) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    }

    /**
     * Checks if the robot is close enough to the target pose.
     */
    private boolean checkDistance(Pose2d targetPose) {
        var pose = getPose();
        if ((Math.hypot(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY()) <= 2.5)) {
            return true;
        }
        return false;
    }

    private Command pathfindCommand(Pose2d targetPose) {
        return Commands.runOnce(() -> {
            var pose = getPose();

            // Check if the robot is already at the target pose.
            if (Math.abs(pose.getX() - targetPose.getX()) <= .1 && Math.abs(pose.getY() - targetPose.getY()) <= .1) {
                return;
            }

            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    getPose(),
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
                    .andThen(swerveAngleCommand(targetPose.getRotation().getDegrees()))
                    .schedule();
        });
    }

    public AutoAlign autoAlign = new AutoAlign();

    public class AutoAlign {
        private boolean errorCheck(Pose2d pose, Rect bounds, CommandXboxController driverController) {
            if (bounds.contains(pose)) {
                return true;
            }
            driverController.getHID().setRumble(RumbleType.kBothRumble, 0.2);
            return false;
        }

        public Command rightSubwooferCommand(CommandXboxController driverController) {
            return runOnce(() -> {
                Pose2d targetPose = Util.flipIfNeeded(new Pose2d(1.185, 6.612, Rotation2d.fromRadians(0.696)));
                if (!errorCheck(getPose(), new Rect(new Pose2d(1.203, 6.261, Rotation2d.fromDegrees(0)), new Pose2d(5.339, 7.737, Rotation2d.fromDegrees(0))), driverController)) {
                    return;
                }
                pathfindCommand(targetPose).schedule();
            });
        }

        public Command leftSubwooferCommand(CommandXboxController driverController) {
            return runOnce(() -> {
                Pose2d pose = Util.flipIfNeeded(new Pose2d(1.195, 4.545, Rotation2d.fromRadians(-1.106)));
                if (!errorCheck(getPose(), new Rect(new Pose2d(1.373, 5.101, Rotation2d.fromDegrees(0)), new Pose2d(2.632, 1.354, Rotation2d.fromDegrees(0))), driverController)) {
                    return;
                }
                pathfindCommand(pose).schedule();
            });
        }

        // public Command frontSubwooferCommand(CommandXboxController driverController) {
        //     Pose2d pose = Util.flipIfNeeded(new Pose2d(1.428, 5.567, Rotation2d.fromDegrees(0)));
        //     if (!errorCheck(pose, new Rect(), driverController)) {
        //         return noop();
        //     }
        //     return pathfindCommand(pose);
        // }

        public Command intakeCommand(CommandXboxController driverController) {
            return runOnce(() -> {
                Pose2d pose = Util.flipIfNeeded(new Pose2d(1.93, 7.716, Rotation2d.fromDegrees(90)));
                if (!errorCheck(getPose(), new Rect(new Pose2d(1.272, 7.681, Rotation2d.fromDegrees(0)), new Pose2d(2.798, 1.455, Rotation2d.fromDegrees(0))), driverController)) {
                    return;
                }
                pathfindCommand(pose).schedule();
            });
        }
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
