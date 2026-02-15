package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import frc.robot.Robot;

@Logged
public class TurretSubsystem extends SubsystemBase {
    private final SwerveSubsystem drive;
    private final SparkMax mTurret = new SparkMax(12, MotorType.kBrushless);

    private final DCMotorSim mTurretSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 4 * 0.15 * 0.15 / 2.0, 10), DCMotor.getNEO(1));

    /**
     * RPM to Volts TODO: Tune
     */
    private SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(0, 0);

    /**
     * Rotations to Volts TODO: Tune
     */
    private ProfiledPIDController turretController = new ProfiledPIDController(0, 0, 0,
            new Constraints(2 * Math.PI, 2 * Math.PI));

    public TurretSubsystem(SwerveSubsystem drive) {
        this.drive = drive;

        if (Robot.isReal()) {
            var turret = new SparkMaxConfig()
                    .idleMode(IdleMode.kBrake);
            turret.softLimit
                    .forwardSoftLimit(200.0 / 20.0 * 0.5) // 180 degrees of motion on a 20:200 reduction
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimit(0)
                    .reverseSoftLimitEnabled(true);

            mTurret.configure(turret, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            mTurret.getEncoder().setPosition(0);
        } else {
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(mTurretSim.getCurrentDrawAmps()));
        }
    }

    /**
     * Returns the turrets angle in the robot's frame of reference.
     * use turretInFieldRelative() if you need the turret's location or angle in
     * field frame.
     * 
     * @return Roation2d
     */
    public Rotation2d getTurretAngle() {
        if (Robot.isSimulation()) {
            return Rotation2d.fromRadians(mTurretSim.getAngularPositionRad());
        } else {
            return Rotation2d.fromRotations(mTurret.getEncoder().getPosition() * 20.0 / 200.0 - 0.25);
        }
    }

    public double getTurretVoltage() {
        if (Robot.isSimulation()) {
            return mTurretSim.getInputVoltage();
        } else {
            return mTurret.getAppliedOutput() * mTurret.getBusVoltage();
        }
    }

    public Pose3d turretInFieldRelative() {
        Pose2d robotPose = drive.getPose();

        // Turret offset in robot frame
        Translation2d launcherOffset = new Translation2d(
                Units.inchesToMeters(29 - 12.5 / 2),
                0);

        // Rotate offset into field frame
        Translation2d fieldRelativeOffset = launcherOffset.rotateBy(robotPose.getRotation());

        // Final field position of turret base
        Translation3d turretTranslation = new Translation3d(
                robotPose.getX() + fieldRelativeOffset.getX(),
                robotPose.getY() + fieldRelativeOffset.getY(),
                Units.inchesToMeters(22));

        // Turret yaw = robot heading + turret relative angle
        double turretYaw = robotPose.getRotation().getRadians()
                + getTurretAngle().getRadians();

        Rotation3d turretRotation = new Rotation3d(0, 0, turretYaw);

        return new Pose3d(turretTranslation, turretRotation);
    }

    /**
     * Computes the angle the turret must rotate (robot-relative)
     * in order to aim at a given field-relative target position.
     *
     * Steps:
     * 1. Compute vector from turret to target (field-relative).
     * 2. Convert that vector into a field-relative angle.
     * 3. Convert field-relative angle into robot-relative angle.
     *
     * @param target Target position in field coordinates (meters)
     * @return Desired turret angle in the robot's reference frame
     */
    public Rotation2d angleToTarget(Translation2d target) {
        // Current robot pose (field-relative)
        Pose2d robotPose = drive.getPose();

        // Current turret pose (field-relative)
        Pose2d turretPose = turretInFieldRelative().toPose2d();

        // Vector from turret to target (still field-relative)
        Translation2d turretToTarget = target.minus(turretPose.getTranslation());

        // Absolute angle from turret to target (field frame)
        Rotation2d fieldRelativeAngleToTarget = turretToTarget.getAngle();

        // Convert field-relative angle into robot-relative angle
        return fieldRelativeAngleToTarget.minus(robotPose.getRotation());
    }

    /**
     * Computes the straight-line distance from the turret
     * to a field-relative target position.
     *
     * @param target
     * @return Distance in meters
     */
    public double distanceToTarget(Translation2d target) {
        Translation2d turretPosition = turretInFieldRelative().getTranslation().toTranslation2d();
        return turretPosition.getDistance(target);
    }

    /**
     * Returns the angular error between where the turret
     * is currently pointing and where it should be pointing.
     *
     * Positive/negative sign indicates direction of error.
     */
    public Rotation2d getTurretAngleError(Translation2d target) {
        return angleToTarget(target).minus(getTurretAngle());
    }

    /**
     * Determines whether the turret is aimed accurately enough to safely shoot at a
     * target. Some targets require different levels of accuracy and the further the
     * away we are the more true we need our aim to be.
     * 
     * @param target             Field Relative target we're aiming at
     * @param maxAcceptableError is the maximum distance we're willing to be off by
     */
    public boolean isTurretAimedWithinTolerance(Translation2d target, double maxAcceptableError) {
        double missDistance = getTurretAngleError(target).getSin() * distanceToTarget(target);
        return Math.abs(missDistance) <= maxAcceptableError;
    }

    /**
     * Aims the turret at the current scoring target.
     *
     * This method:
     * 1. Takes the desired turret angle (in the robot's reference frame).
     * 2. Clamps it to the mechanical limits of the turret.
     * 3. Uses a profiled PID controller to generate feedback voltage.
     * 4. Adds feedforward voltage based on desired velocity.
     * 5. Sends the combined voltage to the turret motor.
     */
    public void aimTurret(Rotation2d robotRelativeRotation) {
        // Clamp to turret mechanical limits (±0.25 rotations = ±90 degrees)
        double clampedGoalRotations = MathUtil.clamp(robotRelativeRotation.getRotations(), -0.25, 0.25);

        // Tell the profiled PID controller our new goal position
        turretController.setGoal(clampedGoalRotations);

        // Feedback (PID) corrects position error
        double feedbackVolts = turretController.calculate(getTurretAngle().getRotations());

        // Feedforward predicts voltage needed to achieve desired velocity
        // The controller's velocity is in rotations/sec but our FF expects
        // rotations/min
        double desiredVelocity = turretController.getSetpoint().velocity * 60.0;
        double feedforwardVolts = turretFF.calculate(desiredVelocity);

        // Combine feedback + feedforward
        if (Robot.isReal()) {
            mTurret.setVoltage(feedbackVolts + feedforwardVolts);
        } else {
            mTurretSim.setInputVoltage(feedbackVolts + feedforwardVolts);
        }
    }

    /**
     * Aims the turret at a field relative target
     * 
     * @param target
     */
    private void aimTurret(Translation2d target) {
        aimTurret(angleToTarget(target));
    }

    public void stop() {
        if (Robot.isReal()) {
            mTurret.stopMotor();
        } else {
            mTurretSim.setInputVoltage(0);
        }
    }

    /**
     * Creates a command that points the turret towards an angle assigned for
     * network tables
     *
     * The dashboard key used is {@code "Turret/targetDegrees"}.
     * The turret will continuously update to match the dashboard value
     * while the command is scheduled.
     */
    public Command cRunTurretSmartDashboard() {
        SmartDashboard.putNumber("Turret/targetDegrees", 0);
        return runEnd(() -> {
            double angle = SmartDashboard.getNumber("Turret/targetDegrees", -90);
            aimTurret(Rotation2d.fromDegrees(angle));
        }, this::stop);
    }

}
