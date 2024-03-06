package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static frc.robot.Util.ifSimElse;
import static frc.robot.Util.switchMode;

public class Intake extends SubsystemBase {
    private final IntakeIO io = switchMode(IntakeIOReal::new, IntakeIOSim::new, IntakeIO::new);
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final TunablePIDController pivotPID = new TunablePIDController("Intake: Pivot", 1, 0, 0);
    private final ArmFeedforward pivotFF = new ArmFeedforward(0, ifSimElse(0.01, 0.0), 0, 0);

    private final LoggedDashboardNumber angle = new LoggedDashboardNumber("Intake: Angle", -IntakeConstants.pivotRadDown);

    private final MechanismLigament2d pivotMechanism = Util.make(() -> {
        var mechanism = new Mechanism2d(6, 6, new Color8Bit(Color.kGray));
        SmartDashboard.putData("Intake", mechanism);
        var root = mechanism.getRoot("Root", 3, 3);
        return root.append(new MechanismLigament2d("Pivot", 3, 0, 4, new Color8Bit(Color.kOrange)));
    });

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);

        pivotMechanism.setAngle(Units.radiansToDegrees(inputs.pivotPositionRad + IntakeConstants.pivotRadDown));

        io.setPivotVoltage(pivotPID.calculate(inputs.pivotPositionRad, angle.get()) + pivotFF.calculate(inputs.pivotPositionRad - IntakeConstants.pivotRadDown, 0));
    }

    public Command moveDown() {
        return run(() -> {
        });
    }
}
