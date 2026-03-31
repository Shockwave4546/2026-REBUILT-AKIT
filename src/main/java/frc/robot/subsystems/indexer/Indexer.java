// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Indexer subsystem for feeding game pieces into the launcher.
 *
 * <p>Simple open-loop voltage control for a single indexer motor. The motor is commanded at full
 * voltage (12V) to feed pieces, or at reduced voltage for reverse/unjam operations.
 */
public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  private boolean isRunning = false;
  private boolean isReversing = false;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Indexer/MotorRPM", inputs.motorVelocityRPM);
    Logger.recordOutput("Indexer/Current", inputs.motorCurrentAmps);

    // Update motor output based on command state
    if (isReversing) {
      io.setDuty(-IndexerConstants.kIndexerVoltage / 12.0);
    } else if (isRunning) {
      io.setDuty(IndexerConstants.kIndexerVoltage / 12.0);
    } else {
      io.setDuty(0.0);
    }

    // Telemetry
    SmartDashboard.putBoolean("Indexer/Running", isRunning);
    SmartDashboard.putBoolean("Indexer/Reversing", isReversing);
    SmartDashboard.putNumber("Indexer/Motor Velocity (RPM)", inputs.motorVelocityRPM);
    SmartDashboard.putNumber("Indexer/Motor Current (A)", inputs.motorCurrentAmps);
    SmartDashboard.putNumber("Indexer/Applied Voltage (V)", inputs.motorAppliedVolts);
  }

  /** Run the indexer at full voltage. */
  public void run() {
    isRunning = true;
    isReversing = false;
  }

  /** Run the indexer in reverse at full voltage. */
  public void runReverse() {
    isRunning = false;
    isReversing = true;
  }

  /** Stop the indexer. */
  public void stop() {
    isRunning = false;
    isReversing = false;
    io.stop();
  }

  /**
   * @return true if the indexer is currently running forward.
   */
  @AutoLogOutput(key = "Indexer/Running")
  public boolean isRunning() {
    return isRunning;
  }

  /**
   * @return true if the indexer is currently running in reverse.
   */
  @AutoLogOutput(key = "Indexer/Reversing")
  public boolean isReversing() {
    return isReversing;
  }
}
