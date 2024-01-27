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

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TunablePIDController;

public class Drivebase extends SubsystemBase {

  private final DrivebaseIO io = chooseIO(
    DrivebaseIOReal::new,
    DrivebaseIOSim::new,
    DrivebaseIO::new
  );
  private final DrivebaseIOInputsAutoLogged inputs = new DrivebaseIOInputsAutoLogged();

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    new Rotation2d(),
    0.0,
    0.0
  );
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
    DrivebaseConstants.trackWidth
  );
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    DrivebaseConstants.feedforwardS,
    DrivebaseConstants.feedforwardV
  );

  private final TunablePIDController swerveModePID = Util.make(() -> {
    var pid = new TunablePIDController(
      "Swerve Mode",
      DrivebaseConstants.swerveModeP,
      0,
      DrivebaseConstants.swerveModeD
    );
    pid.enableContinuousInput(-180, 180);
    return pid;
  });
  private double swerveModeSetpoint = 0;

  public Drivebase() {
    AutoBuilder.configureRamsete(
      this::getPose,
      pose ->
        odometry.resetPosition(
          inputs.gyroYaw,
          getLeftPositionMeters(),
          getRightPositionMeters(),
          pose
        ),
      () ->
        kinematics.toChassisSpeeds(
          new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSec(),
            getRightVelocityMetersPerSec()
          )
        ),
      speeds -> {
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        driveVelocity(
          wheelSpeeds.leftMetersPerSecond,
          wheelSpeeds.rightMetersPerSecond
        );
      },
      new ReplanningConfig(),
      Util::shouldFlip,
      this
    );
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(activePath ->
      Logger.recordOutput(
        "Drivebase/Trajectory",
        activePath.toArray(new Pose2d[activePath.size()])
      )
    );
    PathPlannerLogging.setLogTargetPoseCallback(targetPose ->
      Logger.recordOutput("Drivebase/TrajectorySetpoint", targetPose)
    );
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputs/Drivebase", inputs);

    odometry.update(
      inputs.gyroYaw,
      getLeftPositionMeters(),
      getRightPositionMeters()
    );
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
    Logger.recordOutput(
      "Drivebase/LeftVelocitySetpointMetersPerSec",
      leftMetersPerSec
    );
    Logger.recordOutput(
      "Drivebase/RightVelocitySetpointMetersPerSec",
      rightMetersPerSec
    );
    double leftRadPerSec = leftMetersPerSec / DrivebaseConstants.wheelRadius;
    double rightRadPerSec = rightMetersPerSec / DrivebaseConstants.wheelRadius;
    io.setVelocity(
      leftRadPerSec,
      rightRadPerSec,
      feedforward.calculate(leftRadPerSec),
      feedforward.calculate(rightRadPerSec)
    );
  }

  public Command arcadeDriveCommand(
    CommandXboxController controller,
    boolean preciseMode
  ) {
    if (preciseMode) {
      return run(() ->
        arcadeDrive(
          controller.getLeftY() * DrivebaseConstants.preciseModeMultiplier,
          -controller.getRightX() * DrivebaseConstants.preciseModeMultiplier
        )
      );
    } else {
      return run(() ->
        arcadeDrive(controller.getLeftY(), -controller.getRightX())
      );
    }
  }

  public Command swerveDriveCommand(
    CommandXboxController controller,
    boolean preciseMode
  ) {
    return Commands
      .runOnce(() -> swerveModeSetpoint = getPose().getRotation().getDegrees())
      .andThen(
        run(() -> {
          var x = controller.getRightX();
          var y = controller.getRightY();

          if (
            Math.abs(x) > DrivebaseConstants.swerveModeDeadzone ||
            Math.abs(y) > DrivebaseConstants.swerveModeDeadzone
          ) {
            swerveModeSetpoint = Math.toDegrees(Math.atan2(-x, y));
          } else {
            arcadeDrive(controller.getLeftY(), 0);
          }

          swerveModePID.setSetpoint(swerveModeSetpoint);
          Logger.recordOutput(
            "Drivebase/SwerveMode/Setpoint",
            swerveModeSetpoint
          );
          var robotAngle = getPose().getRotation().getDegrees();
          var rotation = swerveModePID.calculate(robotAngle);
          Logger.recordOutput("Drivebase/SwerveMode/Rotation", rotation);
          if (preciseMode) {
            arcadeDrive(
              controller.getLeftY() * DrivebaseConstants.preciseModeMultiplier,
              rotation
            );
          } else {
            arcadeDrive(controller.getLeftY(), rotation);
          }
        })
      );
  }

  public Command swerveAngleCommand(double angle) {
    return Commands.runOnce(() -> swerveModeSetpoint = angle);
  }

  public Command followPathCommand(String pathName) {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
  }

  private final Runnable noop = new Runnable() {
    @Override
    public void run() {}
  };

  private Command pathFindCommand(Pose2d targetPose) {
    return pathFindCommand(targetPose, targetPose.getRotation());
  }

  // PIDController pathFindPidController;

  /**
   * Goes to given target pose.
   */
  private Command pathFindCommand(
    Pose2d targetPose,
    Rotation2d targetRotation
  ) {
    return runOnce(() -> {
      var pose = getPose();
      if (
        Math.abs(pose.getX() - targetPose.getX()) <= .1 &&
        Math.abs(pose.getY() - targetPose.getY()) <= .1
      ) {
        return;
      }
      if (!(Math.hypot(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY()) <= 2.5)) {
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
        .andThen(turnToTargetCommand(targetRotation))
        .schedule();
    });
    // return new FunctionalCommand(
    //   () -> {
    //     pathFindPidController = new PIDController(
    //       DrivebaseConstants.swerveModeP,
    //       0,
    //       DrivebaseConstants.swerveModeD
    //     );
    //     var pose = getPose();
    //     System.out.println(pose);
    //     pathFindPidController.setSetpoint(
    //       Math.toDegrees(
    //         Math.atan2(
    //           pose.getY() - targetPose.getY(),
    //           pose.getX() - targetPose.getX()
    //         )
    //       )
    //     );
    //     pathFindPidController.setTolerance(5);
    //   },
    //   () -> {
    //     var rotation = pidController.calculate(inputs.gyroYaw.getDegrees());
    //     arcadeDrive(0, rotation);
    //   },
    //   bool -> {
    //     pidController.close();
    //     moveForwardCommand(targetPose).schedule();
    //   },
    //   () -> pidController.atSetpoint(),
    //   this
    // );
  }

  // private Command moveForwardCommand(Pose2d targetPose) {
  //   return new FunctionalCommand(
  //     () -> {
  //       pathFindPidController = new PIDController(1, 0, 0);
  //       pathFindPidController.setSetpoint(0);
  //       pathFindPidController.setTolerance(0.1);
  //       },
  //       () -> {
  //         var pose = getPose();
  //         var speed = pathFindPidController.calculate(
  //           Math.hypot(
  //             pose.getX() - targetPose.getX(),
  //             pose.getY() - targetPose.getY()
  //           )
  //         );
  //         arcadeDrive(speed, 0);
  //       },
  //       interrupted -> {
  //         turnToTargetCommand(targetPose).schedule();
  //       },
  //       () -> pathFindPidController.atSetpoint(),
  //       this
  //     );
  //   }
  // }

  private Command turnToTargetCommand(Rotation2d targetPose) {
    try (
      var pidController = new PIDController(
        DrivebaseConstants.swerveModeP,
        0,
        DrivebaseConstants.swerveModeD
      )
    ) {
      pidController.setSetpoint(targetPose.getDegrees());
      pidController.setTolerance(5);
      return new FunctionalCommand(
        noop,
        () -> {
          var rotation = pidController.calculate(inputs.gyroYaw.getDegrees());
          arcadeDrive(0, rotation);
        },
        bool -> {},
        () -> pidController.atSetpoint(),
        this
      );
    }
  }

  public class AutoAlign {

    public Command rightSubwooferCommand() {
      return pathFindCommand(
        Util.flipIfNeeded(new Pose2d(1.185, 6.612, Rotation2d.fromRadians(0.696)))
      );
    }

    public Command leftSubwooferCommand() {
      return pathFindCommand(
        Util.flipIfNeeded(new Pose2d(1.195, 4.545, Rotation2d.fromRadians(-1.106)))
      );
    }

    public Command intakeSubwooferCommand() {
      return pathFindCommand(
        Util.flipIfNeeded(new Pose2d(1.93, 7.716, Rotation2d.fromDegrees(90)))
      );
    }
  }

  public AutoAlign autoAlign = new AutoAlign();

  @AutoLogOutput
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Command setPoseCommand(Pose2d newPose) {
    return Commands.runOnce(() ->
      odometry.resetPosition(
        inputs.gyroYaw,
        getLeftPositionMeters(),
        getRightPositionMeters(),
        newPose
      )
    );
  }

  public Command resetPoseCommand() {
    return Commands.runOnce(() ->
      odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d())
    );
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
