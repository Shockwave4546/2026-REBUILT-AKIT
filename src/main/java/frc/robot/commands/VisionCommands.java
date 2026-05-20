// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.ShootingConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.launcher.Launcher;
import org.littletonrobotics.junction.Logger;

public class VisionCommands {
  private static final double ANGLE_KP = 3.0;
  private static final double ANGLE_KD = 0.5;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double ANGLE_TOLERANCE = Units.degreesToRadians(2.0);

  // Launcher barrel is angled 3° to the RIGHT of robot forward.
  // Barrel direction in field frame = robotHeading - LAUNCHER_BARREL_ANGLE_RAD.
  // For barrel to face hub: robotHeading - cant = barrelToHubAngle
  //   → desiredHeading = barrelToHubAngle + LAUNCHER_BARREL_ANGLE_RAD
  private static final double LAUNCHER_BARREL_ANGLE_RAD = Math.toRadians(3.0);

  // Barrel is (27/2 - 3.5) = 10 in BEHIND and 10 in RIGHT of robot center.
  // WPILib convention: X = forward, Y = left, so behind = negative X, right = negative Y.
  private static final double LAUNCHER_FORWARD_OFFSET_M =
      -Units.inchesToMeters(10.0); // behind center
  private static final double LAUNCHER_LATERAL_OFFSET_M =
      Units.inchesToMeters(10.0); // right of center

  /**
   * Computes the barrel's position in field coordinates given the current robot pose. Barrel is
   * LAUNCHER_FORWARD_OFFSET_M forward and LAUNCHER_LATERAL_OFFSET_M to the right of robot center.
   */
  private static Translation2d getBarrelWorldPosition(Pose2d robotPose) {
    double heading = robotPose.getRotation().getRadians();
    // Robot forward unit vector: (cos θ, sin θ)
    // Robot right unit vector:   (sin θ, -cos θ)
    double fieldX =
        robotPose.getX()
            + Math.cos(heading) * LAUNCHER_FORWARD_OFFSET_M
            + Math.sin(heading) * LAUNCHER_LATERAL_OFFSET_M;
    double fieldY =
        robotPose.getY()
            + Math.sin(heading) * LAUNCHER_FORWARD_OFFSET_M
            - Math.cos(heading) * LAUNCHER_LATERAL_OFFSET_M;
    return new Translation2d(fieldX, fieldY);
  }

  private VisionCommands() {}

  /**
   * Test command that spins the robot at a constant angular velocity. Useful for verifying that
   * PathPlanner named commands are being called.
   *
   * @param drive The drive subsystem
   * @return A command that spins the robot
   */
  public static Command spin360(Drive drive) {
    System.err.println("[SPIN360 FACTORY] spin360() method called - creating command instance");
    System.out.flush();
    System.err.flush();

    // Counter to reduce logging spam (log every 50 iterations = ~1 second)
    int[] loopCount = {0};

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  System.err.println("[SPIN360] ===== STARTING SPIN360 TEST =====");
                  System.err.flush();
                  Logger.recordOutput("Spin360/Started", true);
                  loopCount[0] = 0;
                }),
            Commands.run(
                    () -> {
                      loopCount[0]++;
                      // Spin at a constant 2 rad/s (about 1 full rotation in 3.14 seconds)
                      if (loopCount[0] % 50 == 0) {
                        System.err.println(
                            "[SPIN360] Spinning at 2.0 rad/s... (iteration " + loopCount[0] + ")");
                        System.err.flush();
                      }
                      ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 2.0);
                      drive.runVelocity(speeds);
                    },
                    drive)
                .withTimeout(3.5), // 3.5 seconds to complete rotation
            Commands.runOnce(
                () -> {
                  System.err.println("[SPIN360] Stopping...");
                  System.err.flush();
                  drive.runVelocity(new ChassisSpeeds());
                  Logger.recordOutput("Spin360/Complete", true);
                },
                drive),
            Commands.runOnce(
                () -> {
                  System.err.println("[SPIN360] ===== SPIN360 TEST COMPLETE =====");
                  System.err.flush();
                }))
        .withName("Spin 360");
  }

  /**
   * Hold command: continuously rotates the robot so the barrel points at the hub and enforces
   * shooting distance (barrel-to-hub). Uses full barrel geometry (position + cant angle). The
   * distance range passed should be in robot-center terms to match the lookup table.
   *
   * @param drive The drive subsystem
   * @param minDistanceM Minimum acceptable barrel-to-hub distance (meters)
   * @param maxDistanceM Maximum acceptable barrel-to-hub distance (meters)
   * @return A command to hold while pressed
   */
  public static Command aimBarrelAtHub(Drive drive, double minDistanceM, double maxDistanceM) {
    final double kAngleKp = 3.0;
    final double kDistKp = 2.0;
    // Robot bumper half-size: module-to-center (0.273m) + bumper thickness (~3.5in = 0.089m)
    final double kBumperHalfSize = 0.273 + Units.inchesToMeters(3.5);
    return Commands.run(
            () -> {
              Pose2d robotPose = drive.getPose();
              boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
              Translation2d hubTarget =
                  isRed ? FieldConstants.Hub.oppCenterPoint : FieldConstants.Hub.centerPoint;

              // --- Rotation: use barrel world position to aim barrel at hub ---
              Translation2d barrelPos = getBarrelWorldPosition(robotPose);
              double bdx = hubTarget.getX() - barrelPos.getX();
              double bdy = hubTarget.getY() - barrelPos.getY();
              double barrelToHubAngle = Math.atan2(bdy, bdx);

              double desiredHeading = barrelToHubAngle + LAUNCHER_BARREL_ANGLE_RAD;
              double headingError = desiredHeading - robotPose.getRotation().getRadians();
              headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
              double angularVelocity = kAngleKp * headingError;

              // --- Translation: enforce distance using robot-CENTER to hub ---
              // minDistanceM/maxDistanceM are in robot-center terms (matching the lookup table).
              double dx = hubTarget.getX() - robotPose.getX();
              double dy = hubTarget.getY() - robotPose.getY();
              double robotCenterDistance = Math.hypot(dx, dy);

              double linearVelocity = 0.0;
              if (robotCenterDistance > maxDistanceM) {
                linearVelocity = kDistKp * (robotCenterDistance - maxDistanceM);
              } else if (robotCenterDistance < minDistanceM) {
                linearVelocity = kDistKp * (robotCenterDistance - minDistanceM);
              }
              linearVelocity = edu.wpi.first.math.MathUtil.clamp(linearVelocity, -1.5, 1.5);

              // --- Alliance zone keepout: bumpers must not cross the alliance zone boundary ---
              // Blue alliance zone: X < 3.4m. Red alliance zone: X > (fieldLength - 3.4m).
              final double kAllianceZoneDepth = 3.4; // meters from each end of field
              double robotX = robotPose.getX();
              if (!isRed) {
                // Blue: front bumper must not exceed the alliance zone boundary
                double limit = kAllianceZoneDepth - kBumperHalfSize;
                if (robotX > limit && linearVelocity > 0) {
                  linearVelocity = 0.0;
                }
              } else {
                // Red: front bumper must not go below the alliance zone boundary
                double limit = FieldConstants.fieldLength - kAllianceZoneDepth + kBumperHalfSize;
                if (robotX < limit && linearVelocity < 0) {
                  linearVelocity = 0.0;
                }
              }

              drive.runVelocity(new ChassisSpeeds(linearVelocity, 0.0, angularVelocity));
            },
            drive)
        .finallyDo(() -> drive.runVelocity(new ChassisSpeeds()));
  }

  /**
   * Aligns the robot's front to the center of the hub using vision. The robot will rotate to face
   * the hub target. Automatically detects which alliance hub based on DriverStation.
   *
   * @param drive The drive subsystem
   * @return A command that aligns the robot to the hub
   */
  public static Command alignToHub(Drive drive) {
    var profiledPidController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    profiledPidController.enableContinuousInput(-Math.PI, Math.PI);
    profiledPidController.setTolerance(ANGLE_TOLERANCE);

    System.err.println(
        "[ALIGN_HUB FACTORY] alignToHub() method called - creating command instance");
    System.err.flush();

    // Counter to reduce logging spam (log every 50 iterations = ~1 second)
    int[] loopCount = {0};
    int[] atSetpointCounter = {0}; // Track how many iterations we've been at setpoint

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              System.err.println("[ALIGN_HUB] ===== STARTING ALIGN TO HUB =====");
              System.err.flush();
              profiledPidController.reset(drive.getPose().getRotation().getRadians());
              loopCount[0] = 0;
              atSetpointCounter[0] = 0;
            }),
        // Main alignment loop - run until we've been at setpoint for several cycles
        Commands.run(
                () -> {
                  loopCount[0]++;
                  // Get current robot position
                  Pose2d robotPose = drive.getPose();

                  // Determine which hub to aim for based on alliance
                  boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                  Translation2d hubTarget =
                      isRed ? FieldConstants.Hub.oppCenterPoint : FieldConstants.Hub.centerPoint;

                  // Compute barrel world position and aim barrel at hub.
                  // desiredHeading = angle(barrel→hub) + cant, because barrel faces
                  // (robotHeading - cant), so: robotHeading - cant = angle → robotHeading = angle +
                  // cant
                  Translation2d barrelPos = getBarrelWorldPosition(robotPose);
                  double bdx = hubTarget.getX() - barrelPos.getX();
                  double bdy = hubTarget.getY() - barrelPos.getY();
                  double barrelToHubAngle = Math.atan2(bdy, bdx);
                  double desiredAngle = barrelToHubAngle + LAUNCHER_BARREL_ANGLE_RAD;

                  // Calculate angular velocity using PID
                  double angularVelocity =
                      profiledPidController.calculate(
                          robotPose.getRotation().getRadians(), desiredAngle);

                  // Log every iteration initially, then less frequently
                  if (loopCount[0] <= 5 || loopCount[0] % 50 == 0) {
                    System.err.printf(
                        "[ALIGN_HUB] Iter %d: Robot: (%.2f, %.2f), RobotAngle: %.2f deg, Target: (%.2f, %.2f), DesiredAngle: %.2f deg, Velocity: %.2f rad/s, AtSetpoint: %s, Counter: %d%n",
                        loopCount[0],
                        robotPose.getX(),
                        robotPose.getY(),
                        Math.toDegrees(robotPose.getRotation().getRadians()),
                        hubTarget.getX(),
                        hubTarget.getY(),
                        Math.toDegrees(desiredAngle),
                        angularVelocity,
                        profiledPidController.atSetpoint(),
                        atSetpointCounter[0]);
                    System.err.flush();
                  }

                  // Track if we're at setpoint (need to be at setpoint for multiple cycles for
                  // stability)
                  if (profiledPidController.atSetpoint()) {
                    atSetpointCounter[0]++;
                  } else {
                    atSetpointCounter[0] = 0; // Reset counter if not at setpoint
                  }

                  // Apply to chassis speeds with zero linear velocity
                  ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, angularVelocity);
                  drive.runVelocity(speeds);
                },
                drive)
            .withTimeout(10.0) // Safety timeout of 10 seconds (shouldn't normally reach this)
            .until(
                () ->
                    atSetpointCounter[0] >= 10), // End when at setpoint for 10 cycles (0.2 seconds)
        // Stop the robot when done
        Commands.runOnce(
            () -> {
              System.err.println(
                  "[ALIGN_HUB] ===== ALIGN TO HUB COMPLETE (after "
                      + loopCount[0]
                      + " iterations) =====");
              System.err.flush();
              drive.runVelocity(new ChassisSpeeds());
            },
            drive));
  }

  /**
   * Enforces a specific distance from the hub by driving forward/backward. The robot will adjust
   * its position to be within the target distance range. Automatically detects which alliance hub
   * based on DriverStation.
   *
   * @param drive The drive subsystem
   * @param minDistance Minimum distance from hub center (meters)
   * @param maxDistance Maximum distance from hub center (meters)
   * @return A command that enforces the distance from the hub
   */
  public static Command enforceDistance(Drive drive, double minDistance, double maxDistance) {
    // PID controller for distance (linear velocity control)
    @SuppressWarnings("resource")
    var pidController = new edu.wpi.first.math.controller.PIDController(2.0, 0.0, 0.1);
    pidController.setTolerance(0.05); // 5cm tolerance

    System.err.println(
        "[DISTANCE_ENFORCER FACTORY] enforceDistance() method called - creating command instance");
    System.err.flush();

    int[] loopCount = {0};
    int[] atSetpointCounter = {0};

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              System.err.println(
                  "[DISTANCE_ENFORCER] ===== STARTING DISTANCE ENFORCEMENT ("
                      + minDistance
                      + "m - "
                      + maxDistance
                      + "m) =====");
              System.err.flush();
              pidController.reset();
              loopCount[0] = 0;
              atSetpointCounter[0] = 0;
            }),
        // Main distance enforcement loop
        Commands.run(
                () -> {
                  loopCount[0]++;
                  Pose2d robotPose = drive.getPose();

                  // Determine which hub to measure distance from
                  boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                  Translation2d hubTarget =
                      isRed ? FieldConstants.Hub.oppCenterPoint : FieldConstants.Hub.centerPoint;

                  // Calculate current distance from hub
                  double dx = hubTarget.getX() - robotPose.getX();
                  double dy = hubTarget.getY() - robotPose.getY();
                  double currentDistance = Math.hypot(dx, dy);

                  // Calculate target distance (middle of the range)
                  double targetDistance = (minDistance + maxDistance) / 2.0;

                  // Calculate linear velocity using PID
                  // When distance > target, we want to move toward hub (positive velocity in
                  // heading direction)
                  // PID outputs negative when error is positive, so we negate it
                  double pidOutput = pidController.calculate(currentDistance, targetDistance);
                  double linearVelocity = -pidOutput; // Negate so positive = move toward hub

                  // Log every iteration initially, then less frequently
                  if (loopCount[0] <= 10 || loopCount[0] % 50 == 0) {
                    System.err.printf(
                        "[DISTANCE_ENFORCER] Iter %d: Robot: (%.2f, %.2f), Distance: %.2f m, Target: %.2f m, HeadingToHub: %.2f deg, Velocity: %.2f m/s, AtSetpoint: %s, Counter: %d%n",
                        loopCount[0],
                        robotPose.getX(),
                        robotPose.getY(),
                        currentDistance,
                        targetDistance,
                        Math.toDegrees(Math.atan2(dy, dx)),
                        linearVelocity,
                        pidController.atSetpoint(),
                        atSetpointCounter[0]);
                    System.err.flush();
                  }

                  // Track if we're at setpoint
                  if (pidController.atSetpoint()) {
                    atSetpointCounter[0]++;
                  } else {
                    atSetpointCounter[0] = 0;
                  }

                  // Drive toward the hub at calculated velocity
                  // Positive velocity = move toward hub, Negative velocity = move away from hub
                  double headingToHub = Math.atan2(dy, dx);
                  ChassisSpeeds speeds =
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          Math.cos(headingToHub) * linearVelocity,
                          Math.sin(headingToHub) * linearVelocity,
                          0.0,
                          drive.getRotation());
                  drive.runVelocity(speeds);
                },
                drive)
            .withTimeout(10.0) // Safety timeout
            .until(() -> atSetpointCounter[0] >= 10), // End when at target distance for 0.2 seconds
        // Stop the robot when done
        Commands.runOnce(
            () -> {
              System.err.println(
                  "[DISTANCE_ENFORCER] ===== DISTANCE ENFORCEMENT COMPLETE (after "
                      + loopCount[0]
                      + " iterations) =====");
              System.err.flush();
              drive.runVelocity(new ChassisSpeeds());
            },
            drive));
  }

  /**
   * Calculates and returns the required flywheel RPM for the current distance from the hub. This
   * can be used by shooting commands to automatically set the correct speed.
   *
   * @param drive The drive subsystem (for getting current pose)
   * @return The required flywheel RPM based on current distance
   */
  public static double getFlywheelRPMForCurrentDistance(Drive drive) {
    Pose2d robotPose = drive.getPose();
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Translation2d hubTarget =
        isRed ? FieldConstants.Hub.oppCenterPoint : FieldConstants.Hub.centerPoint;

    double dx = hubTarget.getX() - robotPose.getX();
    double dy = hubTarget.getY() - robotPose.getY();
    double currentDistance = Math.hypot(dx, dy);

    double requiredRPM = ShootingConstants.getFlywheelRPM(currentDistance);

    // Logging is handled by the caller to avoid spam
    return requiredRPM;
  }

  /**
   * Aligns the robot and shoots at the hub. Combines vision alignment with shooter control.
   *
   * @param drive The drive subsystem
   * @param shootCommand The command to execute for shooting
   * @return A command that aligns to the hub and shoots
   */
  public static Command alignAndShoot(Drive drive, Command shootCommand) {
    return Commands.sequence(alignToHub(drive), shootCommand);
  }

  /** TEST VERSION: Simple setup and shoot that just prints messages. */
  public static Command setupAndShootTest(Drive drive, Launcher launcher, Indexer indexer) {
    System.out.println("\n>>> [SETUP_AND_SHOOT_TEST FACTORY] Test factory method called <<<");
    System.out.flush();
    System.err.println("\n>>> [SETUP_AND_SHOOT_TEST FACTORY] Test factory method called <<<");
    System.err.flush();

    return Commands.waitSeconds(3.0)
        .beforeStarting(
            () -> {
              System.out.println("\n████ [SETUP_SHOOT_TEST] TEST COMMAND STARTING ████");
              System.out.flush();
              System.err.println("\n████ [SETUP_SHOOT_TEST] TEST COMMAND STARTING ████");
              System.err.flush();
            })
        .finallyDo(
            () -> {
              System.out.println("\n████ [SETUP_SHOOT_TEST] TEST COMMAND ENDING ████");
              System.out.flush();
              System.err.println("\n████ [SETUP_SHOOT_TEST] TEST COMMAND ENDING ████");
              System.err.flush();
            })
        .withName("SetupAndShootTest");
  }

  /**
   * Complete setup and shoot command. This command:
   *
   * <ol>
   *   <li>Aligns to hub and enforces distance simultaneously
   *   <li>Starts the shooter and spins up to calculated RPM
   *   <li>Continuously checks safety conditions before indexing:
   *       <ul>
   *         <li>Robot is pointing at hub (within angle tolerance)
   *         <li>Robot is within valid shooting distance
   *         <li>Shooter is at target RPM (within tolerance)
   *       </ul>
   *   <li>If all checks pass, runs indexer to feed and shoot
   *   <li>If any check fails, stops indexer immediately (safety lockout)
   * </ol>
   *
   * <p>The command will timeout after 10 seconds if not interrupted.
   *
   * @param drive The drive subsystem
   * @param launcher The launcher subsystem (flywheel control)
   * @param indexer The indexer subsystem (feeding control)
   * @return A command that sets up position and shoots with safety checks
   */
  public static Command setupAndShoot(Drive drive, Launcher launcher, Indexer indexer) {
    System.out.println("\n>>> [SETUP_AND_SHOOT FACTORY] setupAndShoot() factory method called <<<");
    System.out.flush();

    // Create PID controllers ONCE (not on every periodic call)
    final ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(ANGLE_TOLERANCE);

    @SuppressWarnings("resource")
    final PIDController distanceController = new PIDController(1.0, 0.0, 0.0);
    distanceController.setTolerance(0.05); // 5cm tolerance

    int[] loopCount = {0};
    boolean[] isFiring = {false};
    boolean[] initialized = {false};

    return Commands.run(
            () -> {
              // Initialize on first run
              if (!initialized[0]) {
                angleController.reset(drive.getPose().getRotation().getRadians());
                distanceController.reset();
                loopCount[0] = 0;
                isFiring[0] = false;
                initialized[0] = true;
                System.out.println(
                    "\n\n████████████████████████████████████████████████████████████");
                System.out.println("████ [SETUP_SHOOT] ===== SETUP AND SHOOT STARTED ===== ████");
                System.out.println(
                    "████████████████████████████████████████████████████████████\n");
                System.out.flush();
              }

              loopCount[0]++;

              // Print first real iteration message
              if (loopCount[0] == 1) {
                System.out.println(
                    "\n>>> [SETUP_SHOOT] MAIN LOOP STARTED - Beginning alignment and distance control <<<\n");
                System.out.flush();
              }

              // Get current robot state
              Pose2d robotPose = drive.getPose();
              boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
              Translation2d hubTarget =
                  isRed ? FieldConstants.Hub.oppCenterPoint : FieldConstants.Hub.centerPoint;

              // Robot-center distance — used only for RPM lookup (table was calibrated from center)
              double dx = hubTarget.getX() - robotPose.getX();
              double dy = hubTarget.getY() - robotPose.getY();
              double distanceToHub = Math.hypot(dx, dy);
              double robotHeading = robotPose.getRotation().getRadians();

              // Barrel world position — used for heading and driving distance
              Translation2d barrelPos = getBarrelWorldPosition(robotPose);
              double bdx = hubTarget.getX() - barrelPos.getX();
              double bdy = hubTarget.getY() - barrelPos.getY();
              double barrelToHubAngle = Math.atan2(bdy, bdx);
              double barrelToHubDistance = Math.hypot(bdx, bdy);

              // Desired heading: barrel (which faces robotHeading - cant) must point at hub
              // → robotHeading - cant = barrelToHubAngle → robotHeading = barrelToHubAngle + cant
              double desiredHeading = barrelToHubAngle + LAUNCHER_BARREL_ANGLE_RAD;

              // Get shooting parameters
              double targetRPM = getFlywheelRPMForCurrentDistance(drive);
              double[] distanceRange = ShootingConstants.getDistanceRange();
              double minDistance = distanceRange[0];
              double maxDistance = distanceRange[1];
              double targetDistance = (minDistance + maxDistance) / 2.0;

              // Calculate angle control - pass raw values like alignToHub does
              // enableContinuousInput handles wrapping internally; do NOT pre-compute error
              double angularVelocity = angleController.calculate(robotHeading, desiredHeading);

              // For safety check: get the error the controller computed (handles wrapping)
              double angleDifference = angleController.getPositionError();

              // Distance control: drive based on barrel-to-hub (barrel reaches the right spot).
              // Safety gate + RPM lookup use robot-center distance (matches lookup table).
              double distancePIDOutput =
                  distanceController.calculate(barrelToHubDistance, targetDistance);
              double linearVelocity = -distancePIDOutput;

              // Robot-relative forward/back
              ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity, 0.0, angularVelocity);

              // === SAFETY CHECK: Shooter Status ===
              boolean isAtTargetRPM = launcher.isAtTargetRpm();
              boolean isWithinDistance =
                  distanceToHub >= minDistance && distanceToHub <= maxDistance;
              boolean isPointingAtHub = Math.abs(angleDifference) <= ANGLE_TOLERANCE;

              // === DETERMINE PHASE ===
              // If not yet spinning up, start spinner
              if (!launcher.isSpinningUp() && !launcher.isRunning()) {
                launcher.setTargetRpm(targetRPM);
                launcher.spinUp();
              }

              // === INDEX SAFETY GATE ===
              // Only allow indexer to run if ALL safety checks pass
              boolean shouldFire = isPointingAtHub && isWithinDistance && isAtTargetRPM;
              if (shouldFire) {
                // All good - transition to full run mode and feed
                if (!launcher.isRunning()) {
                  launcher.run();
                }
                indexer.run();
                if (!isFiring[0]) {
                  System.err.println(
                      "\n[SETUP_SHOOT] ==================== FIRING STARTED ====================\n");
                  System.err.flush();
                  isFiring[0] = true;
                }
              } else {
                // Safety lockout - stop indexing immediately
                indexer.stop();
                if (isFiring[0]) {
                  System.err.println(
                      "\n[SETUP_SHOOT] ==================== FIRING STOPPED ====================\n");
                  System.err.flush();
                  isFiring[0] = false;
                }
              }

              // Detailed debug logging (every 10 iterations for minimal spam)
              if (loopCount[0] % 10 == 0 || loopCount[0] <= 3) {
                String debugLog =
                    String.format(
                        "[SETUP_SHOOT] Iter %3d | Angle:%s Dist:%s RPM:%s Fire:%s | "
                            + "Angle:%.1f° CenterDist:%.2fm RPM:%.0f/%.0f Idx:%d",
                        loopCount[0],
                        (isPointingAtHub ? "Y" : "N"),
                        (isWithinDistance ? "Y" : "N"),
                        (isAtTargetRPM ? "Y" : "N"),
                        (isFiring[0] ? "YES" : "no "),
                        Units.radiansToDegrees(angleDifference),
                        distanceToHub,
                        launcher.getShooterRpm(),
                        targetRPM,
                        (indexer.isRunning() ? 1 : 0));
                System.err.println(debugLog);
                System.err.flush();
              }

              // Apply calculated velocities
              drive.runVelocity(speeds);
            },
            drive,
            launcher,
            indexer)
        .withTimeout(10.0)
        .finallyDo(
            () -> {
              if (isFiring[0]) {
                System.err.println(
                    "\n[SETUP_SHOOT] ==================== SETUP AND SHOOT COMPLETE ====================\n");
              } else {
                System.err.println(
                    "\n[SETUP_SHOOT] ==================== SETUP AND SHOOT ABORTED ====================\n");
              }
              System.err.printf("[SETUP_SHOOT] Total iterations: %d%n", loopCount[0]);
              System.err.flush();
              drive.runVelocity(new ChassisSpeeds());
              launcher.stop();
              indexer.stop();
            });
  }
}
