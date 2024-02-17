package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Util;

public final class GeneralConstants {
    /**
     * Automatically determined based on if code is running on a real robot and if {@link SimulationConstants#shouldReplay} is enabled
     */
    public static final Mode mode = RobotBase.isReal() ? Mode.REAL : (SimulationConstants.shouldReplay ? Mode.REPLAY : Mode.SIM);

    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;

    public static final double errorRumbleAmount = 0.75;
    public static final double errorRumbleDuration = 0.25;

    public static final boolean useFileConstants = mode == Mode.SIM;
    public static final double controllerYMultiplier = SimulationConstants.useNintendoSwitchProController ? 1 : -1;

    public static final boolean tuningMode = Util.fileConstant("tuningMode", false);

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
