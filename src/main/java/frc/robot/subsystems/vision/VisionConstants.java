// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "FrontLeft";
  public static String camera1Name = "FrontRight";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // WPILib convention: X = forward (+ = toward front), Y = left (+ = left, - = right), Z = up
  // Front Left Camera: 10.5 inches back, 3 inches right, 20.7 inches high, 15 degrees up
  public static Transform3d robotToCamera0 =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(
              edu.wpi.first.math.util.Units.inchesToMeters(-10.5), // X: 10.5 inches back
              edu.wpi.first.math.util.Units.inchesToMeters(-3.0), // Y: 3 inches right (negative Y)
              edu.wpi.first.math.util.Units.inchesToMeters(20.7) // Z: 20.7 inches high
              ),
          new Rotation3d(0, Math.toRadians(-15), 0) // 15 degrees upward tilt
          );
  // Front Camera 2: 10.5 inches back, 1 inch right (2 inches left of Camera0), 20.7 inches high, 25
  // degrees up
  public static Transform3d robotToCamera1 =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(
              edu.wpi.first.math.util.Units.inchesToMeters(
                  -10.5), // X: 10.5 inches back (same as Camera0)
              edu.wpi.first.math.util.Units.inchesToMeters(
                  -1.0), // Y: 1 inch right (2 inches left of Camera0)
              edu.wpi.first.math.util.Units.inchesToMeters(
                  20.7) // Z: 20.7 inches high (same as Camera0)
              ),
          new Rotation3d(0, Math.toRadians(-25), 0) // 25 degrees upward tilt, forward-facing
          );

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  // Higher values = trust vision less, smoother odometry but slower correction
  // Lower values = trust vision more, faster correction but jitter risk at low speed
  public static double linearStdDevBaseline =
      0.08; // Meters (was 0.02 - increased to reduce jitter)
  public static double angularStdDevBaseline = 0.12; // Radians (was 0.06)

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
