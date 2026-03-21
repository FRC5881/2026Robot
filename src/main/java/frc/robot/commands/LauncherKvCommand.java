package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class LauncherKvCommand extends Command {
    private final LauncherSubsystem launcher;

    private final double[] voltages = { 3, 5, 7, 9 };

    private int index = 0;
    private double kvSum = 0;

    private final double kS = 0.0;

    private Timer timer = new Timer();

    public LauncherKvCommand(LauncherSubsystem launcher) {
        this.launcher = launcher;
    }

    @Override
    public void initialize() {
        index = 0;
        kvSum = 0;
        timer.restart();
    }

    @Override
    public void execute() {
        double v = voltages[index];
        launcher.setVoltage(v);

        if (timer.hasElapsed(2.5)) {
            double rpm = launcher.getLauncherVelocityRPM();
            double kv = (v - kS) / rpm;

            kvSum += kv;

            System.out.println("V=" + v +
                    " RPM=" + rpm +
                    " kV=" + kv);

            index++;
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return index >= voltages.length;
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();;
        double kv = kvSum / voltages.length;
        System.out.println("Average kV = " + kv);
    }

}
