package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class Drive extends Command {
    private SwerveSubsystem swerve;
    private Supplier<ChassisSpeeds> speeds;

    public Drive(SwerveSubsystem swerve, Supplier<ChassisSpeeds> speeds) {
        Preferences.initDouble("Drive Sensitivity", 1.0);
        Preferences.initDouble("Turn Sensitivity", 1.0);

        this.swerve = swerve;
        this.speeds = speeds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ChassisSpeeds targetSpeed = speeds.get();

        targetSpeed.omegaRadiansPerSecond *= Preferences.getDouble("Turn Sensitivity", 1.0);
        targetSpeed.vxMetersPerSecond *= Preferences.getDouble("Drive Sensitivity", 1.0);
        targetSpeed.vyMetersPerSecond *= Preferences.getDouble("Drive Sensitivity", 1.0);

        if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)) {
            targetSpeed.vxMetersPerSecond *= -1;
            targetSpeed.vyMetersPerSecond *= -1;
        }

        swerve.driveFieldOriented(targetSpeed);

        // double maxError = Preferences.getDouble("Turn Max Error (degress)", 45.0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }
}
