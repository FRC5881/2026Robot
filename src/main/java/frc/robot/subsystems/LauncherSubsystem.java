package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

@Logged
public class LauncherSubsystem extends SubsystemBase {
    private final SwerveSubsystem drive;
    private final TurretSubsystem turret;

    public final double MIN_DISTANCE, MAX_DISTANCE;

    private FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(Units.rotationsToRadians(0.30309),
                    Units.rotationsToRadians(0.014391)),
            DCMotor.getNEO(2));
    public Rotation2d simulatedAngle = Rotation2d.kZero;

    private final SparkMax mLauncherMain = new SparkMax(Constants.CANConstants.LAUNCHER_MAIN, MotorType.kBrushless);
    private final SparkMax mLauncherSecondary = new SparkMax(Constants.CANConstants.LAUNCHER_SECONDARY,
            MotorType.kBrushless);

    // kV 0.00206
    // kS 0.15663
    private SimpleMotorFeedforward launcherFF = new SimpleMotorFeedforward(0, 11.0 / 5293.75);
    private PIDController launcherPID = new PIDController(0.0004, 0, 0.001);

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

    private double targetRPM = 0;

    public LauncherSubsystem(SwerveSubsystem drive, TurretSubsystem turret) {
        this.drive = drive;
        this.turret = turret;

        if (Robot.isReal()) {
            var main = new SparkMaxConfig()
                    .idleMode(IdleMode.kCoast)
                    .inverted(false);

            main.closedLoop.pid(0.0003, 0.0, 0.002);
            // main.closedLoop.pid(0.0, 0.0, 0.0);
            main.closedLoop.feedForward
                    .sva(0.1565 - 0.00206 * 60, 0.00206, 0.0);

            mLauncherMain.configure(main, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            var secondary = new SparkMaxConfig()
                    .idleMode(IdleMode.kCoast)
                    .follow(mLauncherMain, true);

            mLauncherSecondary.configure(secondary, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(flywheelSim.getCurrentDrawAmps()));
        }

        // hub tuning
        MIN_DISTANCE = 1.75;
        hubDistanceMap.put(1.75, 2775.0);
        hubDistanceMap.put(2.0, 2900.0);
        hubDistanceMap.put(3.0, 3300.0);
        hubDistanceMap.put(4.0, 3450.0);
        hubDistanceMap.put(5.0, 3800.0);
        hubDistanceMap.put(5.5, 4000.0);
        MAX_DISTANCE = 5.5;

        // passing tuning
        passingDistanceMap.put(0.0, 2000.0);
        passingDistanceMap.put(Units.feetToMeters(9.0), 2600.0);
        passingDistanceMap.put(Units.feetToMeters(16.0), 3500.0);
        passingDistanceMap.put(Units.feetToMeters(23.0), 4400.0);
        passingDistanceMap.put(Units.feetToMeters(100.0), 5500.0);
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
            mLauncherMain.getClosedLoopController().setSetpoint(desiredVelocityRPM, ControlType.kVelocity);
    }

    /**
     * Runs the launcher flywheel at the correct speed for the current target.
     */
    private void runLauncher(Translation2d target, boolean isHub) {
        double velocity;

        // Select the correct distance-to-velocity map depending on which target we are
        // using. Different targets require different trajectories.
        double distanceMeters = turret.distanceToTarget(target);
        SmartDashboard.putNumber("Launcher/distanceToHub", distanceMeters);
        if (isHub) {
            velocity = hubDistanceMap.get(distanceMeters);
        } else {
            velocity = passingDistanceMap.get(distanceMeters);
        }

        targetRPM = velocity;
        runLauncher(velocity);
    }

    public Command cWaitUntilTargetSpeed() {
        return Commands.waitUntil(() -> Math.abs(getLauncherVelocityRPM() - targetRPM) <= 100);
    }

    public void dynamicShoot() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        boolean isRed = alliance == Alliance.Red;

        boolean targetIsHub;
        Translation2d target;
        if (drive.isWithinAllianceZone()) {
            targetIsHub = true;
            target = isRed ? TurretSubsystem.kRedHub : TurretSubsystem.kBlueHub;
        } else {
            targetIsHub = false;
            target = drive.getClosestAllianceCorner();
        }

        runLauncher(target, targetIsHub);
    }

    public Command cRunHub() {
        return runEnd(() -> {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
            Translation2d target = alliance == Alliance.Red ? TurretSubsystem.kRedHub : TurretSubsystem.kBlueHub;
            // runLauncher(target, true);
                        runLauncher(TurretSubsystem.kRedHub, true);

        }, this::stop);
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

    public boolean inTargetRange(Translation2d target, boolean isHub) {
        double distanceMeters = turret.distanceToTarget(target);
        return isHub || (MAX_DISTANCE >= distanceMeters && distanceMeters >= MIN_DISTANCE);
    }

    public void setVoltage(double voltage) {
        if (Robot.isSimulation()) {
            flywheelSim.setInputVoltage(voltage);
        } else {
            mLauncherMain.setVoltage(voltage);
        }
    }

    public Command cRunVelocity(double velocity) {
        return runEnd(() -> runLauncher(velocity), this::stop);
    }

    public Command cRunVoltage(double voltage) {
        return runEnd(() -> setVoltage(voltage), mLauncherMain::stopMotor);
    }

    /**
     * Stops the launcher flywheel.
     */
    public void stop() {
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
                .voltage(Volts.of(mLauncherMain.getBusVoltage() * mLauncherMain.getAppliedOutput()))
                .angularPosition(Rotations.of(mLauncherMain.getEncoder().getPosition()))
                .angularVelocity(RPM.of(mLauncherMain.getEncoder().getVelocity()));
    }, this));

    public Command sysid() {
        return Commands.sequence(
                launcherSysId.quasistatic(Direction.kForward),
                cRunVoltage(0).withTimeout(10),
                launcherSysId.quasistatic(Direction.kReverse),
                cRunVoltage(0).withTimeout(10),
                launcherSysId.dynamic(Direction.kForward),
                cRunVoltage(0).withTimeout(10),
                launcherSysId.dynamic(Direction.kReverse),
                cRunVoltage(0).withTimeout(10));
    }
}
