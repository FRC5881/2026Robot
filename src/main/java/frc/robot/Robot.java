// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

@Logged
public class Robot extends TimedRobot {
  final CommandPS5Controller driver = new CommandPS5Controller(0);
  private final SwerveSubsystem drivebase;
  private final IntakeSubsystem intake;
  private final TurretSubsystem turret;
  private final LauncherSubsystem shooter;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public Robot() {
    Arena2026Rebuilt rebuilt = new Arena2026Rebuilt(false);
    rebuilt.setEfficiencyMode(true);
    rebuilt.resetFieldForAuto();
    SimulatedArena.overrideInstance(rebuilt);

    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    intake = new IntakeSubsystem(drivebase.getMapleSimDrive());
    turret = new TurretSubsystem(drivebase);
    shooter = new LauncherSubsystem(drivebase, turret);

    autoChooser.addOption(
        "Launcher SysId (Quasistatic Forward)",
        shooter.launcherSysId.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Launcher SysId (Quasistatic Reverse)",
        shooter.launcherSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Launcher SysId (Dynamic Forward)",
        shooter.launcherSysId.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Launcher SysId (Dynamic Reverse)",
        shooter.launcherSysId.dynamic(SysIdRoutine.Direction.kReverse));

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX())
        .withControllerRotationAxis(driver::getRightX)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driver.getRawAxis(1),
        () -> -driver.getRawAxis(0))
        .withControllerRotationAxis(() -> -driver.getRawAxis(2))
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .robotRelative(true);

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (Robot.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);

      // X button - run the shooter via network tables & simulate launching a fuel
      // every 1/4 second
      driver.button(2).whileTrue(
          Commands.repeatingSequence(Commands.runOnce(() -> {
            if (intake.obtainFuelFromSim())
              shooter.simLaunchFuel();
          }), Commands.waitSeconds(0.25))
              .alongWith(shooter.cRunLauncherSmartDashboard().alongWith()));
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      // X button - run the shooter via network tables
      driver.cross().whileTrue(shooter.cRunLauncherSmartDashboard());
    }

    driver.L1().whileTrue(Commands.runEnd(intake::runIntake, intake::stopIntake, intake));

    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationInit() {
  }

  StructArrayPublisher<Pose3d> fuelPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("SmartDashboard/Fuel", Pose3d.struct)
      .publish();

  @Override
  public void simulationPeriodic() {
    // Get the positions of fuel (both on the field and in the air)
    Pose3d[] fuels = SimulatedArena.getInstance()
        .getGamePiecesArrayByType("Fuel");

    this.fuelPoses.accept(fuels);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  private Command autonomousCommand;

  @Override
  public void autonomousInit() {
    autonomousCommand = autoChooser.getSelected();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
