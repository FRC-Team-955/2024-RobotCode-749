package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Util.ifSimElse;
import static frc.robot.Util.switchMode;

public class Intake extends SubsystemBase {
    private final IntakeIO io = switchMode(IntakeIOReal::new, IntakeIOSim::new, IntakeIO::new);
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final TunablePIDController pivotPID = Util.make(() -> {
        var p = new TunablePIDController("Intake: Pivot", 2.5, 0, 0);
        p.setTolerance(Units.degreesToRadians(15));
        return p;
    });
    private final ArmFeedforward pivotFF = new ArmFeedforward(0, ifSimElse(0.01, 0.35), 0, 0);
    private final MechanismLigament2d pivotMechanism = Util.make(() -> {
        var mechanism = new Mechanism2d(6, 6, new Color8Bit(Color.kGray));
        SmartDashboard.putData("Intake", mechanism);
        var root = mechanism.getRoot("Root", 3, 3);
        return root.append(new MechanismLigament2d("Pivot", 3, 0, 4, new Color8Bit(Color.kOrange)));
    });
    private boolean usePivotPID = true;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);

        pivotMechanism.setAngle(Units.radiansToDegrees(inputs.pivotPositionRad + IntakeConstants.pivotRadDown));

        var pid = pivotPID.calculate(inputs.pivotPositionRad);
        var ff = pivotFF.calculate(inputs.pivotPositionRad - IntakeConstants.pivotRadDown, inputs.pivotVelocityRadPerSec);
        Logger.recordOutput("Intake/PivotControlSignalPID", pid);
        Logger.recordOutput("Intake/PivotControlSignalFF", ff);
        if (RobotState.isEnabled() && usePivotPID) io.setPivotVoltage(pid + ff);
    }

    private Command pivotPIDToCommand(double setpoint) {
        return startEnd(() -> pivotPID.setSetpoint(setpoint), () -> {
        }).until(pivotPID::atSetpoint);
    }

    public Command intakeCommand() {
        return pivotPIDToCommand(-IntakeConstants.pivotRadDown).andThen(
                startEnd(
                        () -> io.setDriverVoltage(IntakeConstants.intakeSpeed * 12),
                        io::stopDriver
                )
        ).withName("Intake$intake");
    }

    public Command tuckCommand() {
        return pivotPIDToCommand(0).withName("Intake$tuck");
    }

    public Command handoffCommand() {
        return tuckCommand().andThen(
                startEnd(
                        () -> io.setDriverVoltage(IntakeConstants.handoffSpeed * 12),
                        io::stopDriver
                ).withTimeout(IntakeConstants.handoffTimeout)
        ).withName("Intake$handoff");
    }

    public Command ejectCommand() {
        return pivotPIDToCommand(-IntakeConstants.pivotRadEject).andThen(
                startEnd(
                        () -> io.setDriverVoltage(IntakeConstants.ejectSpeed * 12),
                        io::stopDriver
                ).withTimeout(IntakeConstants.ejectTimeout),
                tuckCommand()
        ).withName("Intake$eject");
    }

    public Command pivotSlightlyDownCommand() {
        return startEnd(
                () -> {
                    usePivotPID = false;
                    io.setPivotVoltage(1);
                },
                () -> {
                    usePivotPID = true;
                }
        ).withName("Intake$pivotSlightlyDown");
    }

    public Command resetPivotCommand() {
        return runOnce(io::resetPivotPosition);
    }
}
