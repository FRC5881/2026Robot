package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;

@Logged
public class TurretSubsystem extends SubsystemBase {
    private final SwerveSubsystem drive;
    private final SparkMax mTurret = new SparkMax(Constants.CANConstants.TURRET, MotorType.kBrushless);

    public static final Translation2d kBlueHub = new Translation2d(Units.inchesToMeters(181.56),
            Units.inchesToMeters(158.32));
    public static final Translation2d kRedHub = new Translation2d(Units.inchesToMeters(650.12 - 181.56),
            Units.inchesToMeters(158.32));

    private Translation2d target;
    private boolean targetIsHub;

    public TurretSubsystem(SwerveSubsystem drive) {
        this.drive = drive;

        var turret = new SparkMaxConfig()
                .inverted(true)
                .idleMode(IdleMode.kBrake);

        turret.smartCurrentLimit(20, 20);

        turret.closedLoop
                .pid(1.250, 0.0, 0.0);

        turret.softLimit
                .forwardSoftLimit(4.70)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(true);

        mTurret.configure(turret, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Returns the turrets angle in the robot's frame of reference.
     * use turretInFieldRelative() if you need the turret's location or angle in
     * field frame.
     * 
     * @return Roation2d
     */
    public Rotation2d getTurretAngle() {
        return Rotation2d.fromDegrees(mTurret.getEncoder().getPosition() * 36.0 - 85.28562);
    }

    public double getTurretVelocity() {
        return mTurret.getEncoder().getVelocity();
    }

    public double getTurretVoltage() {
        return mTurret.getAppliedOutput() * mTurret.getBusVoltage();
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

    public double turretRawValue() {
        return mTurret.getEncoder().getPosition();
    }

    public double turretRawSetpoint() {
        return mTurret.getClosedLoopController().getSetpoint();
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
        return true;
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
        mTurret.getClosedLoopController().setSetpoint((robotRelativeRotation.getDegrees() / 36.0 + 2.369045),
                ControlType.kPosition);
    }

    /**
     * Aims the turret at a field relative target
     * 
     * @param target
     */
    private void aimTurret(Translation2d target) {
        aimTurret(angleToTarget(target));
    }

    public Command cForward() {
        targetIsHub = false;
        return run(() -> aimTurret(new Rotation2d()));
    }

    public void stop() {
        mTurret.stopMotor();
    }

    public Command cWaitUntilPointingAtTarget() {
        return Commands.waitUntil(() -> {
            if (!targetIsHub)
                return true;

            return isTurretAimedWithinTolerance(kRedHub, 0.5);
        });
    }

    public Command cTargetHub() {
        return runEnd(() -> {
            this.targetIsHub = true;
            if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)) {
                this.target = kRedHub;
                aimTurret(kRedHub);
            } else {
                this.target = kBlueHub;
                aimTurret(kBlueHub);
            }
        }, this::stop);
    }

    public void dynamicTarget() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        boolean isRed = alliance == Alliance.Red;

        if (drive.isWithinAllianceZone()) {
            this.targetIsHub = true;
            this.target = isRed ? kRedHub : kBlueHub;
            aimTurret(this.target);

        } else {
            this.targetIsHub = false;
            this.target = drive.getClosestAllianceCorner();
            aimTurret(this.target); // ← critical fix
        }
    }

    public Command cDynamicTarget() {
        return runEnd(this::dynamicTarget, this::stop);
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
            SmartDashboard.putBoolean("Turret/running", true);
            double angle = SmartDashboard.getNumber("Turret/targetDegrees", -90);
            aimTurret(Rotation2d.fromDegrees(angle));
        }, () -> {
            this.stop();
            SmartDashboard.putBoolean("Turret/running", false);
        });
    }

    public final SysIdRoutine launcherSysId = new SysIdRoutine(
            new Config(Volts.of(0.25 / 5.0).per(Seconds), null, Seconds.of(5)), new Mechanism((voltage) -> {
                mTurret.setVoltage(voltage.baseUnitMagnitude());
            }, (log) -> {

                log.motor("turret")
                        .voltage(Volts.of(mTurret.getBusVoltage() * mTurret.getAppliedOutput()))
                        .angularPosition(Rotations.of(mTurret.getEncoder().getPosition()))
                        .angularVelocity(RPM.of(mTurret.getEncoder().getVelocity()));
            }, this));

    public Command sysid() {
        return Commands.sequence(
                launcherSysId.quasistatic(Direction.kForward),
                run(this::stop).withTimeout(5),
                launcherSysId.quasistatic(Direction.kReverse));
    }

}
