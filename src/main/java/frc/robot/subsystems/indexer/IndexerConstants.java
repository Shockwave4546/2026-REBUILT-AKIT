// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

/**
 * Constants for the Indexer subsystem.
 *
 * <p>Simple open-loop control constants for feeding game pieces into the launcher.
 */
public final class IndexerConstants {
  // Motor CAN ID
  public static final int kIndexerMotorCanId = 40;

  // Motor inversion
  public static final boolean kIndexerMotorInverted = true;

  // Current limit
  public static final int kIndexerMotorCurrentLimit = 20;

  // Feeder voltage (open-loop, full bus voltage)
  public static final double kIndexerVoltage = 12.0;
}
