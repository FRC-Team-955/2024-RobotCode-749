package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.constants.GeneralConstants;

public class TunablePIDController extends PIDController {
    public TunablePIDController(String name, double kp, double ki, double kd) {
        super(kp, ki, kd);

        if (GeneralConstants.tuningMode) {
            Shuffleboard.getTab("Tunable PID Controllers").add(name, this);
        }
    }
}
