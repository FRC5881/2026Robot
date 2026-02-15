package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

@Logged
public class LauncherSubsystem extends SubsystemBase {
    private final SwerveSubsystem drive;
    private final TurretSubsystem turret;

    private FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2),
                    0.5 * Units.lbsToKilograms(2 * 1.1) * Math.pow(Units.inchesToMeters(2), 2), 1),
            DCMotor.getNEO(2));
    public Rotation2d simulatedAngle = Rotation2d.kZero;

    private final SparkMax mLauncherMain = new SparkMax(10, MotorType.kBrushless);
    private final SparkMax mLauncherSecondary = new SparkMax(11, MotorType.kBrushless);

    /**
     * RPM to Volts TODO: Tune
     */
    private SimpleMotorFeedforward launcherFF = new SimpleMotorFeedforward(0, 11.0 / 5293.75);

    /**
     * RPM to Volts TODO: Tune
     */
    private PIDController launcherPID = new PIDController(0.0005, 0, 0);

    /**
     * Maps distance (meters) to desired launcher velocity (rpm) for scoring in the
     * hub
     */
    private InterpolatingDoubleTreeMap hubDistanceMap = new InterpolatingDoubleTreeMap();

    /**
     * Maps distance (meters) to desired launcher velocity (rpm) for passing to our
     * alliance zone
     */
    private InterpolatingDoubleTreeMap passingDistanceMap = new InterpolatingDoubleTreeMap();

    public LauncherSubsystem(SwerveSubsystem drive, TurretSubsystem turret) {
        this.drive = drive;
        this.turret = turret;

        if (Robot.isReal()) {
            var main = new SparkMaxConfig()
                    .idleMode(IdleMode.kCoast);

            mLauncherMain.configure(main, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            var secondary = new SparkMaxConfig()
                    .idleMode(IdleMode.kCoast)
                    .follow(mLauncherMain, true);

            mLauncherSecondary.configure(secondary, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(flywheelSim.getCurrentDrawAmps()));
        }

        // hub tuning
        hubDistanceMap.put(0.0, 0.0);
        hubDistanceMap.put(5.38, 5600.0);

        // passing tuning
        passingDistanceMap.put(0.0, 0.0);
        passingDistanceMap.put(6.00, 5600.0);
    }

    public double getLauncherVoltage() {
        if (Robot.isSimulation()) {
            return flywheelSim.getInputVoltage();
        } else {
            return mLauncherMain.getAppliedOutput() * mLauncherMain.getBusVoltage();
        }
    }

    public double getLauncherVelocityRPM() {
        if (Robot.isSimulation()) {
            return flywheelSim.getAngularVelocityRPM();
        } else {
            return mLauncherMain.getEncoder().getVelocity();
        }
    }

    /**
     * Runs the launcher flywheel at a defined velocity
     * 
     * @param desiredVelocityRPM
     */
    private void runLauncher(double desiredVelocityRPM) {
        double feedforwardVolts = launcherFF.calculate(desiredVelocityRPM);
        double currentVelocityRPM = flywheelSim.getAngularVelocityRPM();
        double feedbackVolts = launcherPID.calculate(currentVelocityRPM, desiredVelocityRPM);

        if (Robot.isSimulation()) {
            flywheelSim.setInputVoltage(feedforwardVolts + feedbackVolts);
        } else {
            mLauncherMain.setVoltage(feedforwardVolts + feedbackVolts);
        }
    }

    /**
     * Runs the launcher flywheel at the correct speed for the current target.
     */
    private void runLauncher(Translation2d target, boolean isHub) {
        double velocity;

        // Select the correct distance-to-velocity map depending on which target we are
        // using. Different targets require different trajectories.
        double distanceMeters = turret.distanceToTarget(target);
        if (isHub) {
            velocity = hubDistanceMap.get(distanceMeters);
        } else {
            velocity = passingDistanceMap.get(distanceMeters);
        }

        runLauncher(velocity);
    }

    /**
     * Creates a command that runs the launcher using a velocity
     * provided from SmartDashboard.
     *
     * The dashboard key used is {@code "Launcher/targetRPM"}.
     * The launcher will continuously update to match the dashboard value
     * while the command is scheduled.
     */
    public Command cRunLauncherSmartDashboard() {
        SmartDashboard.putNumber("Launcher/targetRPM", 0);
        return runEnd(() -> {
            double velocity = SmartDashboard.getNumber("Launcher/targetRPM", 0);
            runLauncher(velocity);
        }, this::stop);
    }

    /**
     * Stops the launcher flywheel.
     */
    private void stop() {
        if (Robot.isSimulation()) {
            flywheelSim.setInputVoltage(0.0);
        } else {
            mLauncherMain.stopMotor();
        }
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.update(0.02);
    }

    public void simLaunchFuel() {
        if (!Robot.isSimulation())
            return;

        Translation2d launcherOffset = new Translation2d(
                Units.inchesToMeters(29 - 12.5 / 2), 0);

        Pose2d drivetrainPose = drive.getPose();
        ChassisSpeeds fieldRelativeSpeed = drive.getFieldVelocity();

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

    public final SysIdRoutine launcherSysId = new SysIdRoutine(new Config(), new Mechanism((voltage) -> {
        if (Robot.isSimulation()) {
            flywheelSim.setInput(voltage.baseUnitMagnitude());
        } else {
            mLauncherMain.setVoltage(voltage.baseUnitMagnitude());
        }
    }, (log) -> {
        log.motor("launcher")
                .angularPosition(Rotations.of(mLauncherMain.getEncoder().getPosition()))
                .angularVelocity(RPM.of(mLauncherMain.getEncoder().getVelocity()));
    }, this));
}
