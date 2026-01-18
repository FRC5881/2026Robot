package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class IntakeSubsystem extends SubsystemBase {
    private static final int kSimulationMaxFuel = 40;
    private IntakeSimulation intakeSimulation = null;

    public IntakeSubsystem(Optional<SwerveDriveSimulation> mapleSimDrive) {
        if (Robot.isSimulation()) {
            this.intakeSimulation = IntakeSimulation.OverTheBumperIntake("Fuel", mapleSimDrive.get(), Inches.of(25), Inches.of(12), IntakeSide.BACK, kSimulationMaxFuel);
        }
    }

    public void runIntake() {
        if (Robot.isSimulation()) {
            intakeSimulation.startIntake();
        }
    }

    public void stopIntake() {
        if (Robot.isSimulation()) {
            intakeSimulation.stopIntake();
        }
    }

    public boolean obtainFuelFromSim() {
        if (!Robot.isSimulation()) return false;
        return intakeSimulation.obtainGamePieceFromIntake();
    }
}
