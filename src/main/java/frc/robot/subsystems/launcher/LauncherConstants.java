// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launcher;

/**
 * Constants for the Launcher subsystem.
 *
 * <p>Includes hardware configuration (CAN IDs, current limits) and shooter velocity control
 * parameters (PID, feedforward).
 */
public final class LauncherConstants {
  // CAN IDs
  public static final int kFeederMotorCanId = 50; // NEO 550 on SparkMax
  public static final int kShooterLeaderCanId = 51; // Vortex on SparkFlex (leader)
  public static final int kShooterFollowerCanId = 52; // Vortex on SparkFlex (follower, inverted)

  // Motor inversions
  public static final boolean kFeederMotorInverted = false;
  public static final boolean kShooterLeaderInverted = false;
  public static final boolean kShooterFollowerInverted = true; // Physically inverted

  // Current limits (Amps)
  public static final int kFeederMotorCurrentLimit = 20; // NEO 550
  public static final int kShooterMotorCurrentLimit = 70; // Vortex (each)

  // Feeder voltage (open-loop, full bus voltage for max RPM)
  public static final double kFeederVoltage = 12.0;

  // Shooter target RPM
  public static final double kShooterTargetRpm = 4000.0;
  public static final double kShooterShortRpm = 2800.0; // Tunable during matches
  public static final double kShooterLongRpm = 3200.0; // Tunable during matches

  // Shooter PID/FF coefficients (RoboRIO-side control)
  public static final double kP_Shooter = 0.0025;
  public static final double kI_Shooter = 0.0;
  public static final double kD_Shooter = 0.0;
  public static final double kS_Shooter = 0.0;
  public static final double kV_Shooter = 0.00189;
  public static final double kA_Shooter = 0.0;

  // Tolerance for "at target RPM" (RPM)
  public static final double kRpmTolerance = 100.0;

  // Shooter wheel radius (for FuelSim launch velocity calculation)
  // NEO Vortex driving a 4in diameter (2in radius) shooter wheel
  public static final double kShooterWheelRadiusM =
      edu.wpi.first.math.util.Units.inchesToMeters(2.0);

  // Barrel height above ground (for FuelSim launch position)
  public static final double kBarrelHeightM = edu.wpi.first.math.util.Units.inchesToMeters(24.0);

  // Fixed hood angle — shooter launches at ~30° above horizontal
  public static final double kHoodAngleRad = Math.toRadians(30.0);
}
