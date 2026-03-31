// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

/**
 * Constants for the RoboRIO-side profiled intake pivot controller.
 *
 * <p>Includes ProfiledPIDController gains, ArmFeedforward coefficients, and motion profile
 * constraints for the intake arm pivot.
 */
public final class IntakeConstantsProfiled {
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
