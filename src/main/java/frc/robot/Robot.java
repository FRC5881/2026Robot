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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
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
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

  private final SendableChooser<Command> autoChooser;

  public Robot() {
    Arena2026Rebuilt rebuilt = new Arena2026Rebuilt(false);
    rebuilt.setEfficiencyMode(true);
    rebuilt.resetFieldForAuto();
    SimulatedArena.overrideInstance(rebuilt);

    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    intake = new IntakeSubsystem(drivebase.getMapleSimDrive());
    turret = new TurretSubsystem(drivebase);
    shooter = new LauncherSubsystem(drivebase, turret);

    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Drive Reverse", drivebase.driveReverse().withTimeout(2.5));

    SmartDashboard.putData("autoChooser", autoChooser);

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX())
        .withControllerRotationAxis(() -> -driver.getRightX())
        .deadband(OperatorConstants.DEADBAND);

    drivebase.setDefaultCommand(new Drive(drivebase, driveAngularVelocity));
    // turret.setDefaultCommand(turret.cTargetHub());
    turret.setDefaultCommand(turret.cForward());

    driver.L1().whileTrue(intake.cRunIntake());

    Command shootStraightCommand = shooter.cRunVelocity(3000).alongWith(turret.cTargetHub())
        .alongWith(
            Commands.waitSeconds(1.25).andThen(
                indexer.cRun(() -> driver.getHID().getCrossButtonPressed())));

    driver.circle().whileTrue(shootStraightCommand);

    Command shootCommand = shooter.cRunHub()
        .alongWith(turret.cTargetHub())
        .alongWith(
            shooter.cWaitUntilTargetSpeed()
                .andThen(turret.cWaitUntilPointingAtTarget())
                .andThen(indexer.cRun(() -> driver.getHID().getCrossButtonPressed())));

    autoChooser.addOption("Drive Reverse and shoot", drivebase.driveReverse().withTimeout(1.0).andThen(shootCommand));

    Command shootCommand2 = shooter.cRunHub()
        .alongWith(turret.cTargetHub())
        .alongWith(
            shooter.cWaitUntilTargetSpeed()
                .andThen(turret.cWaitUntilPointingAtTarget())
                .andThen(indexer.cRun(() -> driver.getHID().getCrossButtonPressed())));

    driver.R1().whileTrue(shootCommand2);

    driver.povLeft().onTrue(arm.cRun(ArmSubsystem.Target.Extended));
    driver.povUp().onTrue(arm.cRun(ArmSubsystem.Target.Half));
    driver.povRight().onTrue(arm.cRun(ArmSubsystem.Target.Home));

    driver.L3().whileTrue(drivebase.cLock());
    driver.R3().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance, drivebase));

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
    // turret.zero();
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
