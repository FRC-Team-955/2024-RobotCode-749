package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(CommandRobot::new);
    }

    private static class CommandRobot extends TimedRobot {
        private Robot container;
        private Command autonomousCommand;

        public CommandRobot() {
        }

        @Override
        public void robotInit() {
            container = new Robot();
        }

        @Override
        public void robotPeriodic() {
            CommandScheduler.getInstance().run();
        }

        @Override
        public void autonomousInit() {
            autonomousCommand = container.getAutonomousCommand();
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
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
