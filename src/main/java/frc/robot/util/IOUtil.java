package frc.robot.util;

import frc.robot.constants.GeneralConstants;

import java.util.function.Supplier;

public class IOUtil {
    public static <T> T chooseIO(Supplier<T> real, Supplier<T> sim, Supplier<T> replay) {
        switch (GeneralConstants.mode) {
            case REAL -> {
                return real.get();
            }
            case SIM -> {
                return sim.get();
            }
            default -> { // Can't be case REPLAY because of Java restrictions
                return replay.get();
            }
        }
    }
}
