package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public final class GeneralConstants {
    public static final int controllerPort = 0;

    public static final boolean tuningMode = false;

    /**
     * Automatically determined based on if code is running on a real robot and if {@link SimulationConstants#shouldReplay} is enabled
     */
    public static final Mode mode = RobotBase.isReal() ? Mode.REAL : (SimulationConstants.shouldReplay ? Mode.REPLAY : Mode.SIM);

    public enum Mode {
        /**
         * Real robot
         */
        REAL,
        /**
         * Simulation
         */
        SIM,
        /**
         * Log replay
         */
        REPLAY,
    }
}