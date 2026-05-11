// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility for live tuning PID controllers via SmartDashboard.
 *
 * <p>Allows real-time adjustment of PID gains (kP, kI, kD) without code recompilation. Useful for
 * characterizing and tuning motor controllers during practice.
 */
public class PIDTuner {

  /**
   * Update a simple PIDController with values from SmartDashboard.
   *
   * @param controller The PIDController to update
   * @param dashboardPrefix The SmartDashboard prefix (e.g., "Drive/Module0/Steer")
   * @param defaultP Default proportional gain
   * @param defaultI Default integral gain
   * @param defaultD Default derivative gain
   * @return true if any gain was updated
   */
  public static boolean updateSimplePID(
      PIDController controller,
      String dashboardPrefix,
      double defaultP,
      double defaultI,
      double defaultD) {
    // Initialize dashboard entries if they don't exist
    if (!SmartDashboard.containsKey(dashboardPrefix + "/kP")) {
      SmartDashboard.putNumber(dashboardPrefix + "/kP", defaultP);
      SmartDashboard.putNumber(dashboardPrefix + "/kI", defaultI);
      SmartDashboard.putNumber(dashboardPrefix + "/kD", defaultD);
    }

    // Read current values from dashboard
    double p = SmartDashboard.getNumber(dashboardPrefix + "/kP", defaultP);
    double i = SmartDashboard.getNumber(dashboardPrefix + "/kI", defaultI);
    double d = SmartDashboard.getNumber(dashboardPrefix + "/kD", defaultD);

    // Update controller if values changed
    boolean changed = false;
    if (p != controller.getP()) {
      controller.setP(p);
      changed = true;
    }
    if (i != controller.getI()) {
      controller.setI(i);
      changed = true;
    }
    if (d != controller.getD()) {
      controller.setD(d);
      changed = true;
    }

    return changed;
  }

  /**
   * Update a ProfiledPIDController with values from SmartDashboard.
   *
   * @param controller The ProfiledPIDController to update
   * @param dashboardPrefix The SmartDashboard prefix (e.g., "Drive/Module0/Steer")
   * @param defaultP Default proportional gain
   * @param defaultI Default integral gain
   * @param defaultD Default derivative gain
   * @return true if any gain was updated
   */
  public static boolean updateProfiledPID(
      ProfiledPIDController controller,
      String dashboardPrefix,
      double defaultP,
      double defaultI,
      double defaultD) {
    // Initialize dashboard entries if they don't exist
    if (!SmartDashboard.containsKey(dashboardPrefix + "/kP")) {
      SmartDashboard.putNumber(dashboardPrefix + "/kP", defaultP);
      SmartDashboard.putNumber(dashboardPrefix + "/kI", defaultI);
      SmartDashboard.putNumber(dashboardPrefix + "/kD", defaultD);
    }

    // Read current values from dashboard
    double p = SmartDashboard.getNumber(dashboardPrefix + "/kP", defaultP);
    double i = SmartDashboard.getNumber(dashboardPrefix + "/kI", defaultI);
    double d = SmartDashboard.getNumber(dashboardPrefix + "/kD", defaultD);

    // Update controller if values changed
    boolean changed = false;
    if (p != controller.getP()) {
      controller.setP(p);
      changed = true;
    }
    if (i != controller.getI()) {
      controller.setI(i);
      changed = true;
    }
    if (d != controller.getD()) {
      controller.setD(d);
      changed = true;
    }

    return changed;
  }

  /**
   * Publish current PID gains to SmartDashboard for monitoring.
   *
   * @param controller The PIDController to publish
   * @param dashboardPrefix The SmartDashboard prefix
   */
  public static void publishSimplePID(PIDController controller, String dashboardPrefix) {
    SmartDashboard.putNumber(dashboardPrefix + "/kP", controller.getP());
    SmartDashboard.putNumber(dashboardPrefix + "/kI", controller.getI());
    SmartDashboard.putNumber(dashboardPrefix + "/kD", controller.getD());
  }

  /**
   * Publish current PID gains to SmartDashboard for monitoring.
   *
   * @param controller The ProfiledPIDController to publish
   * @param dashboardPrefix The SmartDashboard prefix
   */
  public static void publishProfiledPID(ProfiledPIDController controller, String dashboardPrefix) {
    SmartDashboard.putNumber(dashboardPrefix + "/kP", controller.getP());
    SmartDashboard.putNumber(dashboardPrefix + "/kI", controller.getI());
    SmartDashboard.putNumber(dashboardPrefix + "/kD", controller.getD());
  }
}
