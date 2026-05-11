// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;

/**
 * Live tuning command for individual swerve module control.
 *
 * <p>Allows independent tuning of steering (turn) and drive motors on a single module with
 * independent control of: - Setpoint (target velocity or angle) - Feedforward (kS, kV, kA) - PID
 * gains (P, D)
 *
 * <p>Robot should be on blocks/jacks to allow wheels to spin freely during tuning.
 *
 * <p>Usage: 1. Select this command from auto chooser 2. Enable robot 3. In SmartDashboard, go to
 * "ModuleTuning" tab 4. Set which module (0-3) and whether tuning STEER or DRIVE 5. Adjust Setpoint
 * and FF/PID gains 6. Watch wheel spin or rotate to target angle
 */
public class ModuleTuningCommand extends Command {
  private final Drive drive;
  private final Module[] modules;

  private PIDController steerPID;
  private PIDController drivePID;
  private SimpleMotorFeedforward driveFeedforward;

  public ModuleTuningCommand(Drive drive) {
    this.drive = drive;
    // Get modules via reflection since they're private
    try {
      var modulesField = Drive.class.getDeclaredField("modules");
      modulesField.setAccessible(true);
      this.modules = (Module[]) modulesField.get(drive);
    } catch (Exception e) {
      throw new RuntimeException("Could not access Drive.modules", e);
    }
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Create controllers
    steerPID = new PIDController(0.0, 0.0, 0.0);
    drivePID = new PIDController(0.0, 0.0, 0.0);
    driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

    System.out.println("\n========== MODULE TUNING STARTED ==========");
    System.out.println("Robot must be on blocks/jacks for safe tuning!");
    System.out.println("SmartDashboard path: ModuleTuning/");
    System.out.println("Select:");
    System.out.println("  - Module (0=FL, 1=FR, 2=BL, 3=BR)");
    System.out.println("  - Mode (STEER or DRIVE)");
    System.out.println("  - Then adjust Setpoint and gains");
    System.out.println("==========================================\n");

    // Initialize dashboard
    SmartDashboard.putNumber("ModuleTuning/Module", 0.0); // 0-3
    SmartDashboard.putString("ModuleTuning/Mode", "STEER"); // STEER or DRIVE

    // Steering tuning
    SmartDashboard.putNumber("ModuleTuning/Steer/Setpoint (deg)", 0.0);
    SmartDashboard.putNumber("ModuleTuning/Steer/P", 2.0);
    SmartDashboard.putNumber("ModuleTuning/Steer/D", 0.0);

    // Drive tuning
    SmartDashboard.putNumber("ModuleTuning/Drive/Setpoint (m/s)", 0.0);
    SmartDashboard.putNumber("ModuleTuning/Drive/FF_kS", 0.0);
    SmartDashboard.putNumber("ModuleTuning/Drive/FF_kV", 0.1);
    SmartDashboard.putNumber("ModuleTuning/Drive/FF_kA", 0.0);
    SmartDashboard.putNumber("ModuleTuning/Drive/P", 0.0);
    SmartDashboard.putNumber("ModuleTuning/Drive/D", 0.0);
  }

  @Override
  public void execute() {
    // Read settings from dashboard
    int moduleIndex = (int) SmartDashboard.getNumber("ModuleTuning/Module", 0);
    String mode = SmartDashboard.getString("ModuleTuning/Mode", "STEER");

    // Clamp module index
    moduleIndex = Math.max(0, Math.min(3, moduleIndex));

    Module module = modules[moduleIndex];

    if ("STEER".equals(mode)) {
      tuneSteer(module, moduleIndex);
    } else {
      tuneDrive(module, moduleIndex);
    }
  }

  private void tuneSteer(Module module, int moduleIndex) {
    // Read steering setpoint (degrees → radians)
    double setpointDeg = SmartDashboard.getNumber("ModuleTuning/Steer/Setpoint (deg)", 0.0);
    double setpointRad = Math.toRadians(setpointDeg);

    // Read PID gains
    double p = SmartDashboard.getNumber("ModuleTuning/Steer/P", 2.0);
    double d = SmartDashboard.getNumber("ModuleTuning/Steer/D", 0.0);

    // Get current angle from module
    double currentAngle = module.getAngle().getRadians();

    // Calculate output voltage
    double output = steerPID.calculate(currentAngle, setpointRad);
    output = Math.max(-12.0, Math.min(12.0, output)); // Clamp to 12V

    // Apply to module using SwerveModuleState (keep drive at 0 speed)
    var desiredState =
        new edu.wpi.first.math.kinematics.SwerveModuleState(
            0.0, // Keep drive at 0 speed
            new edu.wpi.first.math.geometry.Rotation2d(setpointRad));
    module.runSetpoint(desiredState);

    // Telemetry
    double error = setpointRad - currentAngle;
    // Normalize to [-π, π]
    while (error > Math.PI) error -= 2.0 * Math.PI;
    while (error < -Math.PI) error += 2.0 * Math.PI;

    SmartDashboard.putNumber("ModuleTuning/Steer/Current (deg)", Math.toDegrees(currentAngle));
    SmartDashboard.putNumber("ModuleTuning/Steer/Error (deg)", Math.toDegrees(error));
    SmartDashboard.putNumber("ModuleTuning/Steer/Output (V)", output);

    // Console output
    System.out.printf(
        "[MOD_%d_STEER] Target:%.1f° Current:%.1f° Error:%.1f° P:%.2f D:%.2f Output:%.2f%n",
        moduleIndex,
        setpointDeg,
        Math.toDegrees(currentAngle),
        Math.toDegrees(error),
        p,
        d,
        output);
  }

  private void tuneDrive(Module module, int moduleIndex) {
    // Read drive setpoint (m/s)
    double setpointVelocity = SmartDashboard.getNumber("ModuleTuning/Drive/Setpoint (m/s)", 0.0);

    // Read FF gains
    double kS = SmartDashboard.getNumber("ModuleTuning/Drive/FF_kS", 0.0);
    double kV = SmartDashboard.getNumber("ModuleTuning/Drive/FF_kV", 0.1);
    double kA = SmartDashboard.getNumber("ModuleTuning/Drive/FF_kA", 0.0);

    // Read PID gains
    double p = SmartDashboard.getNumber("ModuleTuning/Drive/P", 0.0);
    double d = SmartDashboard.getNumber("ModuleTuning/Drive/D", 0.0);

    // Update controllers
    drivePID.setP(p);
    drivePID.setD(d);
    driveFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

    // Get current velocity from module
    double currentVelocity = module.getVelocityMetersPerSec();

    // Calculate output voltage
    double pidOutput = drivePID.calculate(currentVelocity, setpointVelocity);
    double ffOutput = driveFeedforward.calculate(setpointVelocity);
    double totalOutput = pidOutput + ffOutput;
    totalOutput = Math.max(-12.0, Math.min(12.0, totalOutput)); // Clamp to 12V

    // Apply to module using SwerveModuleState
    var desiredState =
        new edu.wpi.first.math.kinematics.SwerveModuleState(
            setpointVelocity, module.getAngle()); // Keep current steering angle
    module.runSetpoint(desiredState);

    // Telemetry
    double error = setpointVelocity - currentVelocity;

    SmartDashboard.putNumber("ModuleTuning/Drive/Current (m/s)", currentVelocity);
    SmartDashboard.putNumber("ModuleTuning/Drive/Error (m/s)", error);
    SmartDashboard.putNumber("ModuleTuning/Drive/FF_Output (V)", ffOutput);
    SmartDashboard.putNumber("ModuleTuning/Drive/PID_Output (V)", pidOutput);
    SmartDashboard.putNumber("ModuleTuning/Drive/Total_Output (V)", totalOutput);

    // Console output (every 10 cycles)
    long cycle = (long) (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 50) % 10;
    if (cycle == 0) {
      System.out.printf(
          "[MOD_%d_DRIVE] Target:%.2f m/s Current:%.2f m/s Error:%.2f m/s "
              + "FF(kS:%.3f kV:%.4f kA:%.4f):%.2fV PID(P:%.3f D:%.3f):%.2fV Total:%.2fV%n",
          moduleIndex,
          setpointVelocity,
          currentVelocity,
          error,
          kS,
          kV,
          kA,
          ffOutput,
          p,
          d,
          pidOutput,
          totalOutput);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all modules
    drive.runVelocity(new edu.wpi.first.math.kinematics.ChassisSpeeds());
    System.out.println("\n========== MODULE TUNING ENDED ==========");
    System.out.println("All modules stopped");
    System.out.println("=========================================\n");
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
