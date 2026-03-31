// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

/**
 * Constants for the Intake subsystem.
 *
 * <p>Includes hardware configuration (CAN IDs, current limits) and control parameters for both
 * the pivot arm and roller motors.
 */
public final class IntakeConstants {
  // Motor CAN IDs
  public static final int kIntakePivotMotorCanId = 30;
  public static final int kIntakeInnerRollerCanId = 31;
  public static final int kIntakeOuterRollerCanId = 32;

  // Motor inversions
  public static final boolean kIntakePivotMotorInverted = true;
  public static final boolean kIntakeInnerRollerInverted = true;
  public static final boolean kIntakeOuterRollerInverted = true;

  // Encoder configuration
  public static final boolean kIntakePivotEncoderInverted = false;

  // Position limits (in rotations)
  public static final double kIntakePivotMinPosition = 0.25; // deployed, hard limit
  public static final double kIntakePivotMaxPosition = 0.66; // retracted, hard limit

  // Named setpoints
  public static final double kIntakePivotDeployedPosition = 0.29;
  public static final double kIntakePivotPartiallyDeployedPosition = 0.367;
  public static final double kIntakePivotWeightlessPosition = 0.58;
  public static final double kIntakePivotRetractedPosition = 0.63;

  // Thresholds for state detection
  public static final double kIntakePivotDeployedThreshold = kIntakePivotDeployedPosition + 0.05;
  public static final double kIntakePivotRetractedThreshold = kIntakePivotRetractedPosition - 0.05;

  // Maximum position for roller to run
  public static final double kIntakeRollerMaxRunPosition = 0.5;

  // Current limits
  public static final int kIntakePivotCurrentLimit = 70;
  public static final int kIntakeRollerCurrentLimit = 20;

  // Roller speeds
  public static final double kIntakeInnerRollerForwardSpeed = 0.8;
  public static final double kIntakeOuterRollerForwardSpeed = 1.0;
  public static final double kIntakeInnerRollerReverseSpeed = -1.0;
  public static final double kIntakeOuterRollerReverseSpeed = -1.0;
  public static final double kIntakeUnjamInnerSpeed = 1.0;
  public static final double kIntakeUnjamOuterSpeed = -1.0;

  // Unjam detection thresholds
  public static final double kUnjamRpmThreshold = 3000.0;
  public static final double kUnjamCurrentThreshold = 18.0;
  public static final double kUnjamDetectionTimeS = 0.2;
  public static final double kUnjamReverseTimeS = 0.4;

  // Pivot current limiting
  public static final double kPivotStallCurrentLimit = 10.0;
  public static final double kPivotStallBackoffDuty = 0.0;

  // Tolerance
  public static final double kTolerance = 0.017;
}
