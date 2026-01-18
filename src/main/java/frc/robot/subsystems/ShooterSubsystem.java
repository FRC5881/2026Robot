package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

@Logged
public class ShooterSubsystem extends SubsystemBase {
    private final FlywheelSim flywheelSim;
    public Rotation2d simulatedAngle;

    public ShooterSubsystem() {
        // (4 inch diameter) 2x 0.8 lb flywheels 2x 0.3 lb wheels
        double r = Units.inchesToMeters(2);
        double m = Units.lbsToKilograms(2 * 1.1);
        double j = 0.5 * m * r * r;

        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), j, 1),
                DCMotor.getNEO(2));

        SimulatedBattery.addElectricalAppliances(() -> Amps.of(flywheelSim.getCurrentDrawAmps()));
    }

    public double velocityRPM() {
        if (Robot.isSimulation()) {
            return flywheelSim.getAngularVelocityRPM();
        } else {
            return 0; // todo real robot
        }
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.update(0.02);
    }

    private static final Translation2d launcherOffset = new Translation2d(
            Units.inchesToMeters(29 - 12.5 / 2), 0);

    public void run(double percent) {
        if (Robot.isSimulation()) {
            flywheelSim.setInputVoltage(SimulatedBattery.getBatteryVoltage().in(Volts) * percent);
        }
    }

    public void setTurretAngle(Rotation2d angle) {
        // in the simulation we're just going to have the turret move instantly
        if (Robot.isSimulation()) {
            this.simulatedAngle = angle;
        }
    }

    public void simLaunchFuel(Pose2d drivetrainPose, ChassisSpeeds fieldRelativeSpeed) {
        if (!Robot.isSimulation())
            return;

        // fuel exit velocity is approx flywheel radius * rotational velocity
        double omega = flywheelSim.getAngularVelocityRadPerSec();
        double radius = Units.inchesToMeters(2);
        double exitVelocity = omega * radius * 0.3;

        SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
                drivetrainPose.getTranslation(),
                launcherOffset,
                fieldRelativeSpeed,
                drivetrainPose.getRotation().plus(this.simulatedAngle),
                Inches.of(22),
                MetersPerSecond.of(exitVelocity),
                Degrees.of(45)));
    }
}