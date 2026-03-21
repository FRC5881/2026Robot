package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class LauncherKsCommand extends Command {
    private final LauncherSubsystem launcher;

    private double voltage = 0.0;
    private final double step = 0.025 * 0.02;

    public double ksResult = 0;

    public LauncherKsCommand(LauncherSubsystem launcher) {
        this.launcher = launcher;
        addRequirements(launcher); 
    }

    @Override
    public void initialize() {
        voltage = 0;
    }

    @Override
    public void execute() {
        launcher.setVoltage(voltage);
        voltage += step;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(launcher.getLauncherVelocityRPM()) > 10;
    }

    @Override
    public void end(boolean interrupted) {
        ksResult = voltage;
        launcher.stop();
        System.out.println("Measured kS = " + ksResult);
    }
}