package frc.robot.subsystems.drivebase.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class SwerveMode {
    private final CommandXboxController driverController;

    private final Drivebase drivebase;

    public SwerveMode(CommandXboxController driverController, Drivebase drivebase) {
        this.driverController = driverController;

        this.drivebase = drivebase;
    }

    private final TunablePIDController swerveModePID = Util.make(() -> {
        var pid = new TunablePIDController("Swerve Mode", DrivebaseConstants.swerveModeP, 0, DrivebaseConstants.swerveModeD);
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(3);
        return pid;
    });
    private double swerveModeSetpoint = 0;

    public Command swerveDriveCommand() {
        return new SwerveDriveCommand();
    }

    public Command swerveAngleCommand(double angle) {
        return Commands.runOnce(() -> swerveModeSetpoint = angle);
    }

    public Command swerveToAngleCommand(double angle) {
        return swerveAngleCommand(angle).andThen(swerveDriveCommand()).until(swerveModePID::atSetpoint);
    }

    private class SwerveDriveCommand extends Command {
        private SwerveDriveCommand() {
            addRequirements(drivebase);
            setName("Drivebase$swerveDrive");
        }

        @Override
        public void initialize() {
            swerveModeSetpoint = drivebase.getPose().getRotation().getDegrees();
        }

        @Override
        public void execute() {
            var reverse = drivebase.getReverseMode() ? -1 : 1;
            var precise = drivebase.getPreciseMode() ? DrivebaseConstants.preciseModeMultiplier : 1;

            var x = reverse * driverController.getLeftX();
            var y = reverse * -driverController.getLeftY();

            if (Math.abs(x) > DrivebaseConstants.swerveModeDeadzone || Math.abs(y) > DrivebaseConstants.swerveModeDeadzone) {
                swerveModeSetpoint = -Math.toDegrees(Math.atan2(x, y));
            }

            swerveModePID.setSetpoint(swerveModeSetpoint);
            Logger.recordOutput("Drivebase/SwerveMode/Setpoint", swerveModeSetpoint);

            var robotAngle = drivebase.getPose().getRotation().getDegrees();
            Logger.recordOutput("Drivebase/SwerveMode/Measurement", robotAngle);

            var speed = precise * reverse * Util.speed(driverController);
            var rotation = precise * swerveModePID.calculate(robotAngle);

            if (GeneralConstants.useControllerDeadzone) {
                if (Math.abs(speed) < GeneralConstants.controllerDeadzone) speed = 0;
            }

            drivebase.arcadeDrive(speed, rotation);
        }
    }
}
