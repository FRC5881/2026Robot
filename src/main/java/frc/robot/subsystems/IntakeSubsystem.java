package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * The intake has 2 motors
 * - mRoller spins the rollers required for picking up fuel
 * - mArm extends and retracts the intake
 */
@Logged
public class IntakeSubsystem extends SubsystemBase {
    private static final int kSimulationMaxFuel = 40;
    private IntakeSimulation intakeSimulation = null;

    public final SparkMax mSpin = new SparkMax(Constants.CANConstants.INTAKE_SPIN, MotorType.kBrushless);

    public IntakeSubsystem(Optional<SwerveDriveSimulation> mapleSimDrive) {
        if (Robot.isSimulation()) {
            this.intakeSimulation = IntakeSimulation.OverTheBumperIntake("Fuel", mapleSimDrive.get(), Inches.of(25), Inches.of(12), IntakeSide.BACK, kSimulationMaxFuel);
        } else {
            var intakeConfig = new SparkMaxConfig()
                .inverted(true)
                .idleMode(IdleMode.kCoast);

            mSpin.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public void runIntake() {
        if (Robot.isSimulation()) {
            intakeSimulation.startIntake();
        } else {
            mSpin.set(1.0);
        }
    }

    public void stopIntake() {
        if (Robot.isSimulation()) {
            intakeSimulation.stopIntake();
        } else {
            mSpin.stopMotor();
        }
    }

    public Command cRunIntake() {
        return startEnd(this::runIntake, this::stopIntake);
    }

    public boolean obtainFuelFromSim() {
        if (!Robot.isSimulation())
            return false;
        return intakeSimulation.obtainGamePieceFromIntake();
    }
}
