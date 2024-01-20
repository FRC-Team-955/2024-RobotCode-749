package frc.robot.subsystems.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.util.IOUtil.chooseIO;

public class Drivebase extends SubsystemBase {
    private final DrivebaseIO io = chooseIO(DrivebaseIOReal::new, DrivebaseIOSim::new, DrivebaseIO::new);
    private final DrivebaseIOInputsAutoLogged inputs = new DrivebaseIOInputsAutoLogged();

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrivebaseConstants.trackWidth);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivebaseConstants.feedforwardS, DrivebaseConstants.feedforwardV);

    public Drivebase() {
        AutoBuilder.configureRamsete(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
                (speeds) -> {
                    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
                    driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
                },
                new ReplanningConfig(),
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Drivebase/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Drivebase/TrajectorySetpoint", targetPose);
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Drivebase", inputs);

        odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters());
    }

    private void arcadeDrive(double speed, double rotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(GeneralConstants.mode == GeneralConstants.Mode.REAL ? rotation : speed, GeneralConstants.mode == GeneralConstants.Mode.REAL ? speed : rotation, true);
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

    public Command arcadeDriveCommand(CommandXboxController controller) {
        return run(() -> arcadeDrive(controller.getLeftY(), -controller.getRightX()));
    }

    public Command followPathCommand(String pathName) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d newPose) {
        odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), newPose);
    }

    public void resetPose() {
        odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
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
