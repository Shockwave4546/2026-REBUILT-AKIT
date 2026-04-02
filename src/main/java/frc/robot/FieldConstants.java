// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains information for location of field elements and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 */
public class FieldConstants {
  // Field dimensions (2026 field is standard FRC dimensions)
  // 54 ft x 27 ft = 16.458m x 8.229m
  public static final double fieldLength = 16.458; // meters
  public static final double fieldWidth = 8.229; // meters

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  public static class LinesVertical {
    public static final double center = fieldLength / 2.0;
    // Hub centers are approximately at these X positions (based on 2026 field layout)
    public static final double hubCenter = 4.5; // Blue alliance hub center X 5.547
    public static final double oppHubCenter = fieldLength - 4.5; // Red alliance hub center X 5.547
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   */
  public static class LinesHorizontal {
    public static final double center = fieldWidth / 2.0;
  }

  /** Hub related constants */
  public static class Hub {
    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(72.0);

    // Center point of hub on alliance side (defined manually to avoid AprilTag loading)
    public static final Translation2d centerPoint =
        new Translation2d(LinesVertical.hubCenter, fieldWidth / 2.0);

    // Opposite side hub
    public static final Translation2d oppCenterPoint =
        new Translation2d(LinesVertical.oppHubCenter, fieldWidth / 2.0);
  }
}
