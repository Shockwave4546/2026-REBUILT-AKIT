// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

/** Contains shooting constants including flywheel RPM lookup table based on distance from hub. */
public class ShootingConstants {
  /**
   * Lookup table for flywheel RPM based on distance from hub.
   *
   * <p>Format: {distance (meters), flywheel RPM}
   *
   * <p>Add more entries as you characterize your shooter at different distances. The system will
   * interpolate between values.
   */
  private static final double[][] RPM_LOOKUP_TABLE = {
    {2.0, 2500}, // 2 meters: 2700 RPM (minimum distance)
    {2.5, 2700},
    {3.0, 2900},
    {3.5, 3400},
    {4.0, 3800}, // 4 meters: 3800 RPM (maximum distance)
  };

  /**
   * Gets the required flywheel RPM for a given distance from the hub. Uses linear interpolation for
   * distances between table entries.
   *
   * @param distanceMeters Distance from hub center in meters
   * @return Required flywheel RPM (interpolated if needed)
   */
  public static double getFlywheelRPM(double distanceMeters) {
    // Clamp distance to valid range
    double distance = Math.max(2.0, Math.min(4.0, distanceMeters));

    // Find the two table entries to interpolate between
    for (int i = 0; i < RPM_LOOKUP_TABLE.length - 1; i++) {
      double dist1 = RPM_LOOKUP_TABLE[i][0];
      double rpm1 = RPM_LOOKUP_TABLE[i][1];
      double dist2 = RPM_LOOKUP_TABLE[i + 1][0];
      double rpm2 = RPM_LOOKUP_TABLE[i + 1][1];

      // If distance falls in this range, interpolate
      if (distance >= dist1 && distance <= dist2) {
        double interpolatedRPM = rpm1 + (rpm2 - rpm1) * (distance - dist1) / (dist2 - dist1);
        return interpolatedRPM;
      }
    }

    // If we get here, distance is beyond the table (shouldn't happen due to clamp)
    return RPM_LOOKUP_TABLE[RPM_LOOKUP_TABLE.length - 1][1];
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
