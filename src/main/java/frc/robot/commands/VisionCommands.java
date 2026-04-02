// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

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
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class VisionCommands {
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double ANGLE_TOLERANCE = Units.degreesToRadians(2.0);

  // Distance enforcer constants
  private static final double DISTANCE_MIN = 1.5; // Test range minimum (meters)
  private static final double DISTANCE_MAX = 3.5; // Test range maximum (meters)
  // For production: use 2.0 and 4.0

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

                  // Calculate desired angle to point at hub center
                  double dx = hubTarget.getX() - robotPose.getX();
                  double dy = hubTarget.getY() - robotPose.getY();
                  double desiredAngle = Math.atan2(dy, dx);

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
   * Aligns the robot and shoots at the hub. Combines vision alignment with shooter control.
   *
   * @param drive The drive subsystem
   * @param shootCommand The command to execute for shooting
   * @return A command that aligns to the hub and shoots
   */
  public static Command alignAndShoot(Drive drive, Command shootCommand) {
    return Commands.sequence(alignToHub(drive), shootCommand);
  }
}
