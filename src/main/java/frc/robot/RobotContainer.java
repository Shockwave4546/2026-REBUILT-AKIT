// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.VisionCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.launcher.LauncherIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused") // Vision feeds pose measurements into drive via addVisionMeasurement
  private final Vision vision;

  private final Intake intake;
  private final Indexer indexer;
  private final Launcher launcher;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Intake toggle state
  private boolean intakeDeployed = false;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        intake =
            new Intake(
                new IntakeIOSparkMax(
                    IntakeConstants.kIntakePivotMotorCanId,
                    IntakeConstants.kIntakeInnerRollerCanId,
                    IntakeConstants.kIntakeOuterRollerCanId));
        indexer = new Indexer(new IndexerIOSparkMax(IndexerConstants.kIndexerMotorCanId));
        launcher =
            new Launcher(
                new LauncherIOSparkMax(
                    LauncherConstants.kFeederMotorCanId,
                    LauncherConstants.kShooterLeaderCanId,
                    LauncherConstants.kShooterFollowerCanId));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        launcher = new Launcher(new LauncherIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        launcher = new Launcher(new LauncherIO() {});
        break;
    }

    // Register named commands for PathPlanner BEFORE building auto chooser
    // Use Suppliers to create new command instances each time they're called
    NamedCommands.registerCommand(
        "Spin 360", Commands.defer(() -> VisionCommands.spin360(drive), java.util.Set.of(drive)));
    NamedCommands.registerCommand(
        "Align to Hub",
        Commands.defer(() -> VisionCommands.alignToHub(drive), java.util.Set.of(drive)));
    // Distance enforcer with test range of 3.5-3.6m
    NamedCommands.registerCommand(
        "Enforce Distance",
        Commands.defer(
            () -> VisionCommands.enforceDistance(drive, 3.5, 3.6), java.util.Set.of(drive)));
    // Setup and shoot (auto version): fully timed, self-terminating —
    //   1. Aim barrel at hub and enforce distance for up to 3s
    //   2. Look up RPM from current barrel distance, spin up (up to 5s to reach RPM)
    //   3. Fire for 8s
    //   4. Stop everything
    NamedCommands.registerCommand(
        "Setup and Shoot",
        Commands.sequence(
            // Phase 1: align barrel and enforce distance (drive required here only)
            VisionCommands.aimBarrelAtHub(drive, 3.09, 3.70).withTimeout(3.0),
            // Phase 2: look up RPM, spin up, fire — no drive required
            Commands.defer(
                () -> {
                  double rpm = VisionCommands.getFlywheelRPMForCurrentDistance(drive);
                  return Commands.sequence(
                      Commands.runOnce(
                          () -> {
                            launcher.setTargetRpm(rpm);
                            launcher.spinUp();
                          },
                          launcher),
                      Commands.waitUntil(launcher::isAtTargetRpm).withTimeout(5.0),
                      Commands.run(
                              () -> {
                                launcher.run();
                                indexer.run();
                              },
                              launcher,
                              indexer)
                          .withTimeout(8.0),
                      Commands.runOnce(
                          () -> {
                            launcher.stop();
                            indexer.stop();
                          },
                          launcher,
                          indexer));
                },
                java.util.Set.of(launcher, indexer))));
    // Aim barrel at hub (align only, no shooting) — same range as X button
    NamedCommands.registerCommand(
        "Aim Barrel at Hub",
        Commands.defer(
            () -> VisionCommands.aimBarrelAtHub(drive, 3.09, 3.70), java.util.Set.of(drive)));
    // Rangefinder shot: read pose distance now, look up RPM, spin up and fire once
    NamedCommands.registerCommand(
        "Rangefinder Shot",
        Commands.defer(
            () -> {
              Pose2d robotPose = drive.getPose();
              boolean isRed =
                  DriverStation.getAlliance()
                      .map(a -> a == DriverStation.Alliance.Red)
                      .orElse(false);
              edu.wpi.first.math.geometry.Translation2d hubTarget =
                  isRed
                      ? frc.robot.FieldConstants.Hub.oppCenterPoint
                      : frc.robot.FieldConstants.Hub.centerPoint;
              edu.wpi.first.math.geometry.Translation2d barrelPos =
                  VisionCommands.getBarrelWorldPosition(robotPose);
              double dist =
                  Math.hypot(
                      hubTarget.getX() - barrelPos.getX(), hubTarget.getY() - barrelPos.getY());
              double rpm = ShootingConstants.getFlywheelRPM(dist);
              return Commands.sequence(
                  Commands.runOnce(() -> launcher.setTargetRpm(rpm), launcher),
                  Commands.runOnce(launcher::spinUp, launcher),
                  Commands.waitUntil(launcher::isAtTargetRpm),
                  Commands.runOnce(indexer::run, indexer),
                  Commands.runOnce(launcher::run, launcher),
                  Commands.waitSeconds(1.0),
                  Commands.runOnce(launcher::stop, launcher),
                  Commands.runOnce(indexer::stop, indexer));
            },
            java.util.Set.of(drive, launcher, indexer)));
    // Press against human player station wall — deploy intake + run rollers while driving back
    NamedCommands.registerCommand(
        "Press Against Wall",
        Commands.parallel(
            DriveCommands.pressAgainstHumanPlayerStation(drive, 0.5, 1.5),
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      intake.setTargetPosition(
                          frc.robot.subsystems.intake.IntakeConstants.kIntakePivotDeployedPosition);
                      intake.run();
                    },
                    intake))));

    // Log that commands are registered
    System.err.println("[RobotContainer] ===== Named Commands Registered =====");
    System.err.println("[RobotContainer] - Spin 360");
    System.err.println("[RobotContainer] - Align to Hub");
    System.err.println("[RobotContainer] - Enforce Distance");
    System.err.println("[RobotContainer] - Setup and Shoot");
    System.err.println("[RobotContainer] - Aim Barrel at Hub");
    System.err.println("[RobotContainer] - Rangefinder Shot");
    System.err.println("[RobotContainer] - Press Against Wall");
    System.err.flush();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    Shuffleboard.getTab("Main")
        .add("Auto", autoChooser.getSendableChooser())
        .withSize(2, 1)
        .withPosition(0, 0);

    Logger.recordOutput("RobotContainer/CommandsRegistered", true);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // A button: Toggle deploy/retract intake
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakeDeployed = !intakeDeployed;
                  if (intakeDeployed) {
                    intake.setTargetPosition(IntakeConstants.kIntakePivotDeployedPosition);
                    intake.run();
                  } else {
                    intake.stopRollers();
                    intake.setTargetPosition(IntakeConstants.kIntakePivotRetractedPosition);
                  }
                },
                intake));

    // Left bumper: Spin intake reverse (unjam)
    controller
        .leftBumper()
        .onTrue(Commands.runOnce(intake::runReverse, intake))
        .onFalse(Commands.runOnce(intake::stop, intake));

    // Right bumper: Short shot - read RPM from HUD
    controller
        .rightBumper()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () ->
                        launcher.setTargetRpm(
                            SmartDashboard.getNumber(
                                "Launcher/Short Shot RPM", LauncherConstants.kShooterShortRpm)),
                    launcher),
                Commands.runOnce(launcher::spinUp, launcher),
                Commands.waitUntil(launcher::isAtTargetRpm),
                Commands.runOnce(indexer::run, indexer),
                Commands.runOnce(launcher::run, launcher)))
        .onFalse(
            Commands.sequence(
                Commands.runOnce(launcher::stop, launcher),
                Commands.runOnce(indexer::stop, indexer)));

    // Left trigger: Long shot - read RPM from HUD
    controller
        .leftTrigger()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () ->
                        launcher.setTargetRpm(
                            SmartDashboard.getNumber(
                                "Launcher/Long Shot RPM", LauncherConstants.kShooterLongRpm)),
                    launcher),
                Commands.runOnce(launcher::spinUp, launcher),
                Commands.waitUntil(launcher::isAtTargetRpm),
                Commands.runOnce(indexer::run, indexer),
                Commands.runOnce(launcher::run, launcher)))
        .onFalse(
            Commands.sequence(
                Commands.runOnce(launcher::stop, launcher),
                Commands.runOnce(indexer::stop, indexer)));

    // Right trigger: Wiggle intake to shuffle pieces into indexer
    controller
        .rightTrigger()
        .onTrue(Commands.runOnce(intake::startWiggle, intake))
        .onFalse(Commands.runOnce(intake::stopWiggle, intake));

    // X button: Hold to aim barrel at hub (pose-based, tag-to-robot geometry)
    controller.x().whileTrue(VisionCommands.aimBarrelAtHub(drive, 3.09, 3.70));

    // D-Pad Up: Full shooting sequence — align barrel, enforce distance, spin up, fire
    controller
        .povUp()
        .whileTrue(
            Commands.defer(
                () -> VisionCommands.setupAndShoot(drive, launcher, indexer),
                java.util.Set.of(drive, launcher, indexer)));

    // D-Pad Down: Rangefinder shot — compute robot-center-to-hub distance from odometry on press,
    // look up RPM, spin up and fire. Uses whileTrue so holding keeps shooter running.
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  // Use barrel-to-hub distance — matches how the lookup table is calibrated.
                  // Falls back to short-shot RPM if alliance is unknown.
                  Pose2d robotPose = drive.getPose();
                  boolean isRed =
                      DriverStation.getAlliance()
                          .map(a -> a == DriverStation.Alliance.Red)
                          .orElse(false);
                  edu.wpi.first.math.geometry.Translation2d hubTarget =
                      isRed
                          ? frc.robot.FieldConstants.Hub.oppCenterPoint
                          : frc.robot.FieldConstants.Hub.centerPoint;
                  edu.wpi.first.math.geometry.Translation2d barrelPos =
                      VisionCommands.getBarrelWorldPosition(robotPose);
                  double dist =
                      Math.hypot(
                          hubTarget.getX() - barrelPos.getX(), hubTarget.getY() - barrelPos.getY());
                  double rpm = ShootingConstants.getFlywheelRPM(dist);
                  System.err.printf("[RANGEFINDER] Barrel dist: %.3f m → RPM: %.0f%n", dist, rpm);
                  launcher.setTargetRpm(rpm);
                  launcher.spinUp();
                },
                launcher))
        .whileTrue(
            Commands.sequence(
                Commands.waitUntil(launcher::isAtTargetRpm),
                Commands.runOnce(indexer::run, indexer),
                Commands.runOnce(launcher::run, launcher),
                Commands.run(() -> {}, launcher, indexer))) // hold until button released
        .onFalse(
            Commands.sequence(
                Commands.runOnce(launcher::stop, launcher),
                Commands.runOnce(indexer::stop, indexer)));

    // B button: Hold to align barrel to hub using global field pose (alignToHub)
    controller
        .b()
        .whileTrue(Commands.defer(() -> VisionCommands.alignToHub(drive), java.util.Set.of(drive)));

    // Start button: Zero gyro heading — 0° on blue (facing red wall), 180° on red (facing blue
    // wall)
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      boolean isRed =
                          DriverStation.getAlliance()
                              .map(a -> a == DriverStation.Alliance.Red)
                              .orElse(false);
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              Rotation2d.fromDegrees(isRed ? 180.0 : 0.0)));
                    },
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Called every robot loop. Publishes live shooting telemetry to SmartDashboard. */
  public void periodic() {
    Pose2d robotPose = drive.getPose();
    boolean isRed =
        DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
    edu.wpi.first.math.geometry.Translation2d hubTarget =
        isRed ? FieldConstants.Hub.oppCenterPoint : FieldConstants.Hub.centerPoint;
    edu.wpi.first.math.geometry.Translation2d barrelPos =
        VisionCommands.getBarrelWorldPosition(robotPose);
    double distToHub =
        Math.hypot(hubTarget.getX() - barrelPos.getX(), hubTarget.getY() - barrelPos.getY());
    double lookupRPM = ShootingConstants.getFlywheelRPM(distToHub);

    SmartDashboard.putNumber("Shooting/Distance to Hub (m)", distToHub);
    SmartDashboard.putNumber("Shooting/Distance to Hub (in)", Units.metersToInches(distToHub));
    SmartDashboard.putNumber("Shooting/Lookup RPM", lookupRPM);
  }
}
