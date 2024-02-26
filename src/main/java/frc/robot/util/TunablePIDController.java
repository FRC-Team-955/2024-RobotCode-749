package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class TunablePIDController extends PIDController {
    public TunablePIDController(String name, double kp, double ki, double kd) {
        super(kp, ki, kd);

        Shuffleboard.getTab("Tunable PID Controllers").add(name, this);
    }
}
