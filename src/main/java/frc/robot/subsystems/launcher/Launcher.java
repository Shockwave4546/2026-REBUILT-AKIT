// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Launcher subsystem with RoboRIO-side PID control for the shooter wheels.
 *
 * <p>The subsystem consists of:
 *
 * <ul>
 *   <li>Feeder: NEO 550 on SparkMax, open-loop voltage control to feed pieces into shooter
 *   <li>Shooter Leader: Vortex on SparkFlex, velocity control via RoboRIO-side PID + feedforward
 *   <li>Shooter Follower: Vortex on SparkFlex, hardware-slaved to leader with inverted output
 * </ul>
 *
 * <p>The feeder and indexer only engage once the shooter has reached target RPM to prevent jamming.
 */
public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputs inputs = new LauncherIOInputs();

  // Shooter velocity control (RoboRIO-side)
  private final SimpleMotorFeedforward shooterFF;
  private final PIDController shooterPID;

  private boolean isRunning = false;
  private boolean isSpinningUp = false;
  private boolean isFeederRunning = false;
  private boolean isFeederReversing = false;
  private boolean feederLatched = false; // latches true once feeder fires; reset on stop()

  private double targetRpm = LauncherConstants.kShooterTargetRpm;
  // TODO: These will be used for short/long shot commands

  public Launcher(LauncherIO io) {
    this.io = io;

    // Initialize feedforward and PID for shooter velocity control
    shooterFF =
        new SimpleMotorFeedforward(
            LauncherConstants.kS_Shooter,
            LauncherConstants.kV_Shooter,
            LauncherConstants.kA_Shooter);

    shooterPID =
        new PIDController(
            LauncherConstants.kP_Shooter,
            LauncherConstants.kI_Shooter,
            LauncherConstants.kD_Shooter);

    // Seed dashboard entries
    SmartDashboard.putNumber("Launcher/Short Shot RPM", LauncherConstants.kShooterShortRpm);
    SmartDashboard.putNumber("Launcher/Long Shot RPM", LauncherConstants.kShooterLongRpm);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Launcher/ShooterRPM", inputs.shooterLeaderVelocityRPM);
    Logger.recordOutput("Launcher/FeederCurrent", inputs.feederCurrentAmps);

    // Update shooter control
    updateShooterControl();

    // Telemetry
    SmartDashboard.putBoolean("Launcher/Running", isRunning);
    SmartDashboard.putBoolean("Launcher/Spinning Up", isSpinningUp);
    SmartDashboard.putNumber("Launcher/Shooter RPM", inputs.shooterLeaderVelocityRPM);
    SmartDashboard.putNumber("Launcher/Target RPM", targetRpm);
    SmartDashboard.putBoolean("Launcher/At Target RPM", isAtTargetRpm());
  }

  private void updateShooterControl() {
    if (isRunning || isSpinningUp) {
      // Calculate voltage using PID + feedforward
      double pidOutput = shooterPID.calculate(inputs.shooterLeaderVelocityRPM, targetRpm);
      double ffOutput = shooterFF.calculate(targetRpm);
      double voltage = pidOutput + ffOutput;

      // Clamp to 12V bus
      voltage = Math.max(-12.0, Math.min(12.0, voltage));
      io.setShooterVoltage(voltage);

      // Debug telemetry
      SmartDashboard.putNumber("Launcher/PID Output", pidOutput);
      SmartDashboard.putNumber("Launcher/FF Output", ffOutput);
      SmartDashboard.putNumber("Launcher/Applied Voltage", voltage);
    } else {
      io.setShooterVoltage(0.0);
      shooterPID.reset();
    }

    // Handle feeder based on shooter state.
    // Latch: once the feeder fires at target RPM, keep it running through any RPM dip.
    if (isRunning) {
      if (isAtTargetRpm() && isFeederRunning) {
        feederLatched = true;
      }
      if (isFeederReversing) {
        io.setFeederDuty(-LauncherConstants.kFeederVoltage / 12.0);
      } else if (feederLatched) {
        io.setFeederDuty(LauncherConstants.kFeederVoltage / 12.0);
      } else {
        io.setFeederDuty(0.0);
      }
    } else {
      io.setFeederDuty(0.0);
    }

    // Feeder diagnostics
    SmartDashboard.putNumber("Launcher/Feeder RPM", inputs.feederVelocityRPM);
    SmartDashboard.putNumber("Launcher/Feeder Current (A)", inputs.feederCurrentAmps);
  }

  /** Run the launcher (spins up shooter and feeds when ready). */
  public void run() {
    isRunning = true;
    isSpinningUp = false;
    isFeederRunning = true;
    isFeederReversing = false;
  }

  /** Spin up the shooter without feeding. */
  public void spinUp() {
    isSpinningUp = true;
    isRunning = false;
    isFeederRunning = false;
    isFeederReversing = false;
  }

  /** Stop the launcher. */
  public void stop() {
    isRunning = false;
    isSpinningUp = false;
    isFeederRunning = false;
    isFeederReversing = false;
    feederLatched = false;
    io.stop();
  }

  /** Run feeder in reverse (for unjamming). */
  public void reverseFeeder() {
    isFeederReversing = true;
    isFeederRunning = false;
  }

  /** Stop feeder. */
  public void stopFeeder() {
    isFeederRunning = false;
    isFeederReversing = false;
  }

  /** Set target RPM for shooter. */
  public void setTargetRpm(double rpm) {
    targetRpm = rpm;
  }

  /**
   * @return true if shooter is at target RPM.
   */
  @AutoLogOutput(key = "Launcher/AtTargetRPM")
  public boolean isAtTargetRpm() {
    return Math.abs(inputs.shooterLeaderVelocityRPM - targetRpm) <= LauncherConstants.kRpmTolerance;
  }

  /**
   * @return current shooter RPM.
   */
  public double getShooterRpm() {
    return inputs.shooterLeaderVelocityRPM;
  }

  /**
   * @return true if launcher is running.
   */
  public boolean isRunning() {
    return isRunning;
  }

  /**
   * @return true if launcher is currently spinning up (running but not feeding).
   */
  public boolean isSpinningUp() {
    return isSpinningUp;
  }

  /**
   * @return the current target RPM for the shooter.
   */
  public double getTargetRpm() {
    return targetRpm;
  }

  /**
   * @return true once the feeder has fired (latched on after RPM + feeder both active). Reset by
   *     stop(). Use in auto to detect when a shot has been committed.
   */
  public boolean isFeederLatched() {
    return feederLatched;
  }
}
