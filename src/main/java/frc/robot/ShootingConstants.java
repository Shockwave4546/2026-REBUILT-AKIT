// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

/** Contains shooting constants including flywheel RPM lookup table based on distance from hub. */
public class ShootingConstants {

  // -----------------------------------------------------------------------
  // Fudge factors — adjust at events without touching the lookup table
  // -----------------------------------------------------------------------

  /**
   * Distance offset (meters). Added to the measured camera distance before looking up RPM. Use a
   * positive value if the robot consistently undershoots (acts closer than it is), negative if it
   * overshoots. Start at 0.0 and tune in small increments (~0.05 m).
   */
  public static double kDistanceOffsetMeters = 0.0;

  /**
   * RPM multiplier. Applied to the interpolated RPM from the lookup table. Use > 1.0 for
   * harder/bouncier balls that need more speed, < 1.0 for softer balls. Typical range: 0.90 – 1.10.
   */
  public static double kRpmMultiplier = 1.0;

  // -----------------------------------------------------------------------
  /**
   * Lookup table for flywheel RPM based on distance from hub.
   *
   * <p>Format: {distance (meters), flywheel RPM}
   *
   * <p>Add more entries as you characterize your shooter at different distances. The system will
   * interpolate between values.
   */
  private static final double[][] RPM_LOOKUP_TABLE = {
    {3.09, 2675}, // 3.09 m → 2675 RPM (minimum calibrated distance)
    {3.22, 2725},
    {3.41, 2800},
    {3.70, 2900} //  3.70 m → 2900 RPM (maximum calibrated distance)
  };

  /**
   * Gets the required flywheel RPM for a given distance from the hub. Uses linear interpolation for
   * distances between table entries.
   *
   * @param distanceMeters Distance from hub center in meters
   * @return Required flywheel RPM (interpolated if needed)
   */
  public static double getFlywheelRPM(double distanceMeters) {
    double minDist = RPM_LOOKUP_TABLE[0][0];
    double maxDist = RPM_LOOKUP_TABLE[RPM_LOOKUP_TABLE.length - 1][0];

    // Apply distance offset fudge factor, then clamp to table range
    double distance = Math.max(minDist, Math.min(maxDist, distanceMeters + kDistanceOffsetMeters));

    // Find the two table entries to interpolate between
    for (int i = 0; i < RPM_LOOKUP_TABLE.length - 1; i++) {
      double dist1 = RPM_LOOKUP_TABLE[i][0];
      double rpm1 = RPM_LOOKUP_TABLE[i][1];
      double dist2 = RPM_LOOKUP_TABLE[i + 1][0];
      double rpm2 = RPM_LOOKUP_TABLE[i + 1][1];

      if (distance >= dist1 && distance <= dist2) {
        double interpolatedRPM = rpm1 + (rpm2 - rpm1) * (distance - dist1) / (dist2 - dist1);
        // Apply RPM multiplier fudge factor
        return interpolatedRPM * kRpmMultiplier;
      }
    }

    // If we get here, distance is beyond the table (shouldn't happen due to clamp)
    return RPM_LOOKUP_TABLE[RPM_LOOKUP_TABLE.length - 1][1] * kRpmMultiplier;
  }

  /**
   * Gets the RPM for a specific distance without interpolation. Returns the exact RPM if the
   * distance is in the table, otherwise returns the closest value.
   *
   * @param distanceMeters Distance from hub center in meters
   * @return Required flywheel RPM
   */
  public static double getFlywheelRPMClosest(double distanceMeters) {
    double closestDistance = RPM_LOOKUP_TABLE[0][0];
    double closestRPM = RPM_LOOKUP_TABLE[0][1];
    double minDifference = Math.abs(distanceMeters - closestDistance);

    for (double[] entry : RPM_LOOKUP_TABLE) {
      double difference = Math.abs(distanceMeters - entry[0]);
      if (difference < minDifference) {
        minDifference = difference;
        closestDistance = entry[0];
        closestRPM = entry[1];
      }
    }

    return closestRPM;
  }

  /**
   * Gets information about the valid shooting distance range.
   *
   * @return Array containing [minDistance, maxDistance]
   */
  public static double[] getDistanceRange() {
    return new double[] {RPM_LOOKUP_TABLE[0][0], RPM_LOOKUP_TABLE[RPM_LOOKUP_TABLE.length - 1][0]};
  }
}
