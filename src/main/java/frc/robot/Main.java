package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.lang.reflect.Array;

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(CommandRobot::new);
    }

    private static class CommandRobot extends LoggedRobot {
        private Robot container;
        private Command autonomousCommand;
        private Command teleopInitCommand;

        public CommandRobot() {
        }

        private void logConstantClass(Class<?> clazz, String parentName) {
            var parent = (parentName != null ? parentName + "." : "");
            for (var field : clazz.getFields()) {
                var key = parent + clazz.getSimpleName() + "." + field.getName();
                try {
                    var value = field.get(null);
                    if (value.getClass().isArray()) {
                        for (int i = 0; i < Array.getLength(value); i++) {
                            Logger.recordMetadata(key + "[" + i + "]", Array.get(value, i).toString());
                        }
                    } else {
                        Logger.recordMetadata(key, value.toString());
                    }
                } catch (IllegalAccessException | IllegalArgumentException e) {
                    Logger.recordMetadata(key, "Unknown");
                }
            }
            for (var subclass : clazz.getClasses()) {
                logConstantClass(subclass, parent + clazz.getSimpleName());
            }
        }

        @Override
        public void robotInit() {
            Logger.recordMetadata("* ProjectName", BuildConstants.MAVEN_NAME);
            Logger.recordMetadata("* BuildDate", BuildConstants.BUILD_DATE);
            Logger.recordMetadata("* GitSHA", BuildConstants.GIT_SHA);
            Logger.recordMetadata("* GitDate", BuildConstants.GIT_DATE);
            Logger.recordMetadata("* GitBranch", BuildConstants.GIT_BRANCH);
            switch (BuildConstants.DIRTY) {
                case 0:
                    Logger.recordMetadata("* GitDirty", "All changes committed");
                    break;
                case 1:
                    Logger.recordMetadata("* GitDirty", "Uncommitted changes");
                    break;
                default:
                    Logger.recordMetadata("* GitDirty", "Unknown");
                    break;
            }

            // By doing this, we also initialize file constants and any other variable constants
            logConstantClass(Constants.class, null);

            switch (Constants.mode) {
                case REAL -> {
                    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                    Logger.addDataReceiver(new NT4Publisher()); // Log to NetworkTables
                    SmartDashboard.putData("PowerDistribution", new PowerDistribution(Constants.pdhId, PowerDistribution.ModuleType.kRev)); // Enables power distribution logging
                }
                case SIM -> {
                    Logger.addDataReceiver(new NT4Publisher());
                }
                case REPLAY -> {
                    setUseTiming(false); // Run as fast as possible
                    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                    Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                }
            }

            Logger.start();

            container = new Robot();
        }

        @Override
        public void robotPeriodic() {
            CommandScheduler.getInstance().run();
        }

        @Override
        public void teleopInit() {
            teleopInitCommand = container.getTeleopInitCommand();
            if (teleopInitCommand != null) teleopInitCommand.schedule();
        }

        @Override
        public void teleopExit() {
            if (teleopInitCommand != null) {
                teleopInitCommand.cancel();
                teleopInitCommand = null;
            }
        }

        @Override
        public void autonomousInit() {
            autonomousCommand = container.getAutonomousCommand();
            if (autonomousCommand != null) autonomousCommand.schedule();
        }

        @Override
        public void autonomousExit() {
            if (autonomousCommand != null) {
                autonomousCommand.cancel();
                autonomousCommand = null;
            }
        }
    }
}
