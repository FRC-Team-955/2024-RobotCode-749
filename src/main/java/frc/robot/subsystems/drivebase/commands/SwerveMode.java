package frc.robot.subsystems.drivebase.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class SwerveMode {
    private final Drivebase drivebase;

    public SwerveMode(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

    private static final TunablePIDController swerveModePID = Util.make(() -> {
        var pid = new TunablePIDController("Swerve Mode", DrivebaseConstants.swerveModeP, 0, DrivebaseConstants.swerveModeD);
        pid.enableContinuousInput(-180, 180);
        return pid;
    });
    private static double swerveModeSetpoint = 0;

    public Command swerveDriveCommand(CommandXboxController controller) {
        return Commands
                .runOnce(() -> swerveModeSetpoint = drivebase.getPose().getRotation().getDegrees())
                .andThen(drivebase.run(() -> {
                    var x = (drivebase.getReverseMode() ? -1 : 1) * controller.getRightX();
                    var y = (drivebase.getReverseMode() ? -1 : 1) * controller.getRightY() * (SimulationConstants.useNintendoSwitchProController ? 1 : -1);

                    if (Math.abs(x) > DrivebaseConstants.swerveModeDeadzone || Math.abs(y) > DrivebaseConstants.swerveModeDeadzone) {
                        swerveModeSetpoint = Math.toDegrees(Math.atan2(-x, y));
                    }

                    swerveModePID.setSetpoint(swerveModeSetpoint);
                    Logger.recordOutput("Drivebase/SwerveMode/Setpoint", swerveModeSetpoint);
                    var robotAngle = drivebase.getPose().getRotation().getDegrees();
                    var rotation = swerveModePID.calculate(robotAngle);
                    Logger.recordOutput("Drivebase/SwerveMode/Rotation", rotation);
                    if (drivebase.getPreciseMode()) {
                        drivebase.arcadeDrive(controller.getLeftY() * DrivebaseConstants.preciseModeMultiplier, rotation);
                    } else {
                        drivebase.arcadeDrive(controller.getLeftY(), rotation);
                    }
                }));
    }

    public Command swerveAngleCommand(double angle) {
        return Commands.runOnce(() -> swerveModeSetpoint = angle);
    }
}
