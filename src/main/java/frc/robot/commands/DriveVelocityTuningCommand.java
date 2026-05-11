// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.PIDTuner;

/**
 * Live tuning command for testing and optimizing drive velocity PID gains.
 *
 * <p>This command lets you: 1. Adjust drive velocity PID gains (kP, kI, kD) via SmartDashboard in
 * real-time 2. Command the robot to drive at a target velocity (forward) 3. See live feedback of
 * velocity error, acceleration, and PID output 4. Useful for tuning feedforward (kS, kV) as well
 *
 * <p>How to use: 1. Run this command on the field (with space to drive forward) 2. Open
 * SmartDashboard 3. Look for "DriveVelTuner/" tab 4. Set Target Velocity (m/s) - start small like
 * 0.5 5. Adjust PID gains and watch velocity converge 6. Look for smooth acceleration without
 * overshoot
 *
 * <p>NOTE: This tests the robot's drive velocity response. For feedforward tuning, see Drive
 * subsystem characterization notes.
 */
public class DriveVelocityTuningCommand extends Command {
  private final Drive drive;
  private PIDController controller;
  private double targetVelocity = 0.5; // m/s

  public DriveVelocityTuningCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Create controller with initial drive gains
    controller = new PIDController(DriveConstants.driveKp, 0.0, DriveConstants.driveKd);
    controller.setTolerance(0.05); // 5 cm/s tolerance

    System.out.println("\n========== DRIVE VELOCITY TUNING STARTED ==========");
    System.out.println("Use SmartDashboard to adjust: DriveVelTuner/Target Velocity (m/s)");
    System.out.println(
        "Adjust PID gains in real-time: DriveVelTuner/kP, DriveVelTuner/kI, DriveVelTuner/kD");
    System.out.println("Watch velocity ramp up smoothly without overshoot");
    System.out.println("====================================================\n");

    // Initialize dashboard
    SmartDashboard.putNumber("DriveVelTuner/Target Velocity (m/s)", 0.5);
    SmartDashboard.putNumber("DriveVelTuner/kP", DriveConstants.driveKp);
    SmartDashboard.putNumber("DriveVelTuner/kI", 0.0);
    SmartDashboard.putNumber("DriveVelTuner/kD", DriveConstants.driveKd);
  }

  @Override
  public void execute() {
    // Read target velocity from dashboard
    targetVelocity = SmartDashboard.getNumber("DriveVelTuner/Target Velocity (m/s)", 0.5);

    // Live tune PID gains
    PIDTuner.updateSimplePID(
        controller, "DriveVelTuner", DriveConstants.driveKp, 0.0, DriveConstants.driveKd);

    // Get estimated velocity from SmartDashboard (updated by velocity observation)
    double estimatedVelocity = SmartDashboard.getNumber("DriveVelTuner/Est Velocity", 0.0);

    // Calculate PID output (additional voltage to add to feedforward)
    double pidOutput = controller.calculate(estimatedVelocity, targetVelocity);

    // Apply to robot - drive forward at target velocity
    // Use feedforward as base + PID for correction
    double ffOutput = DriveConstants.driveKv * targetVelocity;
    double totalOutput = ffOutput + pidOutput;

    // Command forward velocity (only X-axis for linear testing)
    ChassisSpeeds speeds = new ChassisSpeeds(totalOutput, 0.0, 0.0);
    drive.runVelocity(speeds);

    // Publish telemetry
    double velocityError = controller.getPositionError();
    SmartDashboard.putNumber("DriveVelTuner/Velocity Error (m/s)", velocityError);
    SmartDashboard.putNumber("DriveVelTuner/PID Output (V)", pidOutput);
    SmartDashboard.putNumber("DriveVelTuner/FF Output (V)", ffOutput);
    SmartDashboard.putNumber("DriveVelTuner/Total Output (V)", totalOutput);
    SmartDashboard.putBoolean("DriveVelTuner/At Setpoint", controller.atSetpoint());

    // Periodic console output
    int loopCount = (int) (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 50) % 50;
    if (loopCount == 0) {
      System.out.printf(
          "[DRIVE_VEL_TUNER] Target: %.2f m/s | Est: %.2f m/s | Error: %.2f m/s | FF: %.2f V | PID: %.2f V | P:%.4f D:%.4f%n",
          targetVelocity,
          estimatedVelocity,
          velocityError,
          ffOutput,
          pidOutput,
          controller.getP(),
          controller.getD());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
    System.out.println("\n========== DRIVE VELOCITY TUNING ENDED ==========");
    System.out.println("Final PID Gains:");
    System.out.printf(
        "  kP = %.6f%n  kI = %.6f%n  kD = %.6f%n",
        controller.getP(), controller.getI(), controller.getD());
    System.out.println("====================================================\n");
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
