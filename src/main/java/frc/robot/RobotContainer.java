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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    // Setup and shoot: aligns, enforces distance, and shoots with safety checks
    NamedCommands.registerCommand(
        "Setup and Shoot",
        Commands.defer(
            () -> VisionCommands.setupAndShoot(drive, launcher, indexer),
            java.util.Set.of(drive, launcher, indexer)));

    // Log that commands are registered
    System.err.println("[RobotContainer] ===== Named Commands Registered =====");
    System.err.println("[RobotContainer] - Spin 360");
    System.err.println("[RobotContainer] - Align to Hub");
    System.err.println("[RobotContainer] - Enforce Distance");
    System.err.println("[RobotContainer] - Setup and Shoot");
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

    // X button: Hold to point at AprilTag and enforce shooting distance (46–77.5 in)
    // Camera 0 is 10.5 in behind robot center, so camera distance = robot center distance + 10.5 in
    final double kCameraOffsetM = Units.inchesToMeters(10.5);
    final double kMinDistanceM = Units.inchesToMeters(46.0) + kCameraOffsetM;
    final double kMaxDistanceM = Units.inchesToMeters(77.5) + kCameraOffsetM;
    final double kAngleKp = 3.0;
    final double kDistKp = 2.0;
    controller
        .x()
        .whileTrue(
            Commands.run(
                () -> {
                  // --- Rotation: servo onto tag yaw ---
                  Rotation2d targetX = vision.getTargetX(0);
                  double angularVelocity = -kAngleKp * targetX.getRadians();

                  // --- Translation: enforce distance only outside the allowed range ---
                  double distance = vision.getTargetDistance(0);
                  double linearVelocity = 0.0;
                  if (distance > 0) {
                    if (distance > kMaxDistanceM) {
                      // Too far — drive forward toward tag
                      linearVelocity = kDistKp * (distance - kMaxDistanceM);
                    } else if (distance < kMinDistanceM) {
                      // Too close — back away from tag
                      linearVelocity = kDistKp * (distance - kMinDistanceM);
                    }
                    linearVelocity = MathUtil.clamp(linearVelocity, -1.5, 1.5);
                  }

                  // Drive forward/back in robot-relative X (camera faces forward)
                  drive.runVelocity(new ChassisSpeeds(linearVelocity, 0.0, angularVelocity));
                },
                drive))
        .onFalse(Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive));

    // Reset gyro to 180° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(), Rotation2d.fromDegrees(180))),
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
}
