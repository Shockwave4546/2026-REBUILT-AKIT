// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launcher;

public interface LauncherIO {
  /** Updates the set of loggable inputs. */
  default void updateInputs(LauncherIOInputs inputs) {}

  /** Set feeder motor duty cycle. */
  default void setFeederDuty(double duty) {}

  /** Set shooter leader motor voltage. */
  default void setShooterVoltage(double volts) {}

  /** Stop all launcher motors. */
  default void stop() {}
}
