// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // ============================================================================
  // INTAKE SUBSYSTEM CONSTANTS
  // ============================================================================
  public static final class IntakeConstants {
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

  // ============================================================================
  // INTAKE PROFILED CONTROLLER CONSTANTS
  // ============================================================================
  public static final class IntakeConstantsProfiled {
    // TrapezoidProfile constraints
    public static final double kMaxVelocity = 600 * 0.00278; // rot/s
    public static final double kMaxAcceleration = 200 * 0.00278; // rot/s²

    // ProfiledPID gains
    public static final double kP = 3.5;
    public static final double kI = 0.0;
    public static final double kD = 0.1;

    // Feedforward gains
    public static final double kS = 0.00125;
    public static final double kG = 0.02;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    // Gravity geometry
    public static final double kGravityPeakPosition = 0.29;
    public static final double kGravityZeroPosition = 0.58;

    // Tolerance
    public static final double kTolerance = 0.017;
  }
}
