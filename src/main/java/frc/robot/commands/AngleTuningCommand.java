// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.PIDTuner;

/**
 * Live tuning command for testing and optimizing rotation PID gains.
 *
 * <p>This command lets you: 1. Adjust angle PID gains (kP, kI, kD) via SmartDashboard in real-time
 * 2. Command the robot to rotate to a target angle 3. See live feedback of position error,
 * velocity, and PID output 4. No recompile needed - tune and test immediately!
 *
 * <p>How to use: 1. Run this command on the field 2. Open SmartDashboard 3. Look for "AngleTuner/"
 * tab 4. Adjust Target Angle and PID gains 5. Watch the "Position Error" and "Output Velocity"
 * plots 6. Tune for smooth, fast settling without overshoot
 */
public class AngleTuningCommand extends Command {
  private final Drive drive;

  private ProfiledPIDController controller;
  private double targetAngle = 0.0; // radians

  public AngleTuningCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Create controller with initial gains from constants
    controller =
        new ProfiledPIDController(
            DriveConstants.turnKp,
            0.0,
            DriveConstants.turnKd,
            new TrapezoidProfile.Constraints(8.0, 20.0)); // Same constraints as angle controller
    controller.enableContinuousInput(-Math.PI, Math.PI);

    System.out.println("\n========== ANGLE TUNING STARTED ==========");
    System.out.println(
        "Use SmartDashboard to adjust: AngleTuner/Target Angle (radians or degrees)");
    System.out.println(
        "Adjust PID gains in real-time: AngleTuner/kP, AngleTuner/kI, AngleTuner/kD");
    System.out.println("=========================================\n");

    // Initialize dashboard
    SmartDashboard.putNumber("AngleTuner/Target Angle (deg)", 0.0);
    SmartDashboard.putNumber("AngleTuner/kP", DriveConstants.turnKp);
    SmartDashboard.putNumber("AngleTuner/kI", 0.0);
    SmartDashboard.putNumber("AngleTuner/kD", DriveConstants.turnKd);
  }

  @Override
  public void execute() {
    // Read target angle from dashboard (in degrees, convert to radians)
    double targetAngleDeg = SmartDashboard.getNumber("AngleTuner/Target Angle (deg)", 0.0);
    targetAngle = Math.toRadians(targetAngleDeg);

    // Live tune PID gains
    PIDTuner.updateProfiledPID(
        controller, "AngleTuner", DriveConstants.turnKp, 0.0, DriveConstants.turnKd);

    // Get current robot angle
    double currentAngle = drive.getPose().getRotation().getRadians();

    // Calculate output
    double outputVelocity = controller.calculate(currentAngle, targetAngle);

    // Apply to robot
    drive.runVelocity(new edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, outputVelocity));

    // Publish telemetry
    double positionError = controller.getPositionError();
    SmartDashboard.putNumber("AngleTuner/Current Angle (deg)", Math.toDegrees(currentAngle));
    SmartDashboard.putNumber("AngleTuner/Position Error (deg)", Math.toDegrees(positionError));
    SmartDashboard.putNumber("AngleTuner/Output Velocity (rad/s)", outputVelocity);
    SmartDashboard.putBoolean("AngleTuner/At Setpoint", controller.atSetpoint());

    // Periodic console output (every 50 iterations = ~1 second)
    int loopCount = (int) (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 50) % 50;
    if (loopCount == 0) {
      System.out.printf(
          "[ANGLE_TUNER] Target: %.1f° | Current: %.1f° | Error: %.1f° | Output: %.2f rad/s | P:%.3f D:%.3f%n",
          Math.toDegrees(targetAngle),
          Math.toDegrees(currentAngle),
          Math.toDegrees(positionError),
          outputVelocity,
          controller.getP(),
          controller.getD());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new edu.wpi.first.math.kinematics.ChassisSpeeds());
    System.out.println("\n========== ANGLE TUNING ENDED ==========");
    System.out.println("Final PID Gains:");
    System.out.printf(
        "  kP = %.4f%n  kI = %.4f%n  kD = %.4f%n",
        controller.getP(), controller.getI(), controller.getD());
    System.out.println("=========================================\n");
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
