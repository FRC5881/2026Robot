package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

@Logged
public class ShooterSubsystem extends SubsystemBase {
    private final FlywheelSim flywheelSim;
    public Rotation2d simulatedAngle = Rotation2d.kZero;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 11.0 / 5293.75);
    private PIDController controller = new PIDController(0.0005, 0, 0);
    private double targetRPM = 3130.0;

    private final Pose2d kHub = new Pose2d(
            new Translation2d(4.03 + 1.19 / 2, 8.07 / 2),
            Rotation2d.kZero);

    private final SwerveSubsystem drive;

    public ShooterSubsystem(SwerveSubsystem drive) {
        this.drive = drive;

        // (4 inch diameter) 2x 0.8 lb flywheels 2x 0.3 lb wheels
        double r = Units.inchesToMeters(2);
        double m = Units.lbsToKilograms(2 * 1.1);
        double j = 0.5 * m * r * r;

        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), j, 1),
                DCMotor.getNEO(2));

        SimulatedBattery.addElectricalAppliances(() -> Amps.of(flywheelSim.getCurrentDrawAmps()));
    }

    public Pose3d turrentInFieldRelative() {
        Pose2d pose = drive.getPose();

        Translation3d launcher = new Translation3d(pose.getX(), pose.getY(), Units.inchesToMeters(22));
        Rotation3d rotation = new Rotation3d(pose.getRotation().plus(simulatedAngle));

        return new Pose3d(launcher, rotation);
    }

    public double getVoltage() {
        if (Robot.isSimulation()) {
            return flywheelSim.getInputVoltage();
        } else {
            return 0; // todo real robot
        }
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
        solveTurretDistance(kHub.getTranslation());

        simulatedAngle = solveTurretAngle(kHub.getTranslation());
        flywheelSim.update(0.02);
    }

    private static final Translation2d launcherOffset = new Translation2d(
            Units.inchesToMeters(29 - 12.5 / 2), 0);

    public void run(double velocityRPM) {
        targetRPM = velocityRPM;

        if (Robot.isSimulation()) {
            double ff = feedforward.calculate(targetRPM);
            double pid = controller.calculate(flywheelSim.getAngularVelocityRPM(), targetRPM);
            flywheelSim.setInputVoltage(ff + pid);
        }else{
            double ff = feedforward.calculate(targetRPM);
            double pid = controller.calculate(flywheelSim.getAngularVelocityRPM(), targetRPM);
            flywheelSim.setInputVoltage(ff + pid);
        }
    }

    public void stop() {
        flywheelSim.setInputVoltage(0);
    }

    public void setTurretAngle(Rotation2d angle) {
        // in the simulation we're just going to have the turret move instantly
        if (Robot.isSimulation()) {
            this.simulatedAngle = angle;
        }
    }

    public void simLaunchFuel() {
        if (!Robot.isSimulation())
            return;

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

    /**
     * 
     * @param drivePose Robot's translation and rotation
     * @param target    Target to point the turret at
     * @return
     */
    public Rotation2d solveTurretAngle(Translation2d target) {
        Pose2d drivePose = drive.getPose();

        // Robot (x, y) d
        Translation2d driveTranslation = drivePose.getTranslation();

        double robot_x = driveTranslation.getX();
        double robot_y = driveTranslation.getY();
        double target_x = target.getX();
        double target_y = target.getY();

        double a = robot_y - target_y;
        double b = robot_x - target_x;

        double diff_rad = -drivePose.getRotation().getRadians() + Math.atan2(a, b) + Math.PI;

        return new Rotation2d(diff_rad);
    }

    public Distance solveTurretDistance(Translation2d target) {
        double distance = drive.getPose().getTranslation().getDistance(target);
        SmartDashboard.putNumber("Shooter/DistanceToHub", distance);
        return Meters.of(distance);
    }

    public double calculateExitVelocityHub(double targetDistanceMeters, double targetHeightMeters){//double targetDistanceMeters, double targetHeightMeters) {

        InterpolatingDoubleTreeMap insane = new InterpolatingDoubleTreeMap();
        insane.put(5.38,5600.0);
        //update data as get 

        double distanceH = Math.sqrt((targetDistanceMeters*targetDistanceMeters)+((72-24)*(72-24)));

        double result = insane.get(distanceH);

        double velocityW = result*(Math.PI/2)*(5.91/2);
        double velocityE = velocityW*.5;

        return velocityE;

    }

    public double calculateExitVelocityPass(double targetDistanceMeters, double targetHeightMeters){//double targetDistanceMeters, double targetHeightMeters) {

        InterpolatingDoubleTreeMap insane = new InterpolatingDoubleTreeMap();
        //insane.put(0,0.0);
        //update data as get 

        double result = insane.get(targetDistanceMeters);

        double velocityW = result*(Math.PI/2)*(5.91/2);
        double velocityE = velocityW*.5;

        return velocityE;

    }
}
