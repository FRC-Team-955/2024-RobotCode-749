package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TunablePIDController extends PIDController implements LoggableInputs {
    private final String name;

    public TunablePIDController(String name, double kp, double ki, double kd) {
        super(kp, ki, kd);
        this.name = name;

        Shuffleboard.getTab("Tunable PID Controllers").add(name, this);
        process();
    }

    private void process() {
        Logger.processInputs("Inputs/TunablePIDControllers/" + name, this);
    }


    @Override
    public void toLog(LogTable table) {
        table.put("P", getP());
        table.put("I", getI());
        table.put("D", getD());
        table.put("IZone", getIZone());
        table.put("PosTol", getPositionTolerance());
        table.put("VelTol", getVelocityTolerance());
    }

    @Override
    public void fromLog(LogTable table) {
        System.out.printf(
                "TunablePIDControllers do not listen to log changes; this one (%s):\n- P = %d\n- I = %d\n- D = %d\n- IZone = %d\n- PosTol = %d\n- VelTol = %d",
                name,
                table.get("P", -1),
                table.get("I", -1),
                table.get("D", -1),
                table.get("IZone", -1),
                table.get("PosTol", -1),
                table.get("VelTol", -1)
        );
    }

    @Override
    public void setP(double kp) {
        super.setP(kp);
        process();
    }

    @Override
    public void setI(double ki) {
        super.setI(ki);
        process();
    }

    @Override
    public void setD(double kd) {
        super.setD(kd);
        process();
    }

    @Override
    public void setIZone(double iZone) {
        super.setIZone(iZone);
        process();
    }

    @Override
    public void setSetpoint(double setpoint) {
        super.setSetpoint(setpoint);
        process();
    }

    @Override
    public void setTolerance(double positionTolerance) {
        super.setTolerance(positionTolerance);
        process();
    }

    @Override
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        super.setTolerance(positionTolerance, velocityTolerance);
        process();
    }
}
