// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

public interface IntakeIO {
  /** Updates the set of loggable inputs. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Set pivot motor duty cycle. */
  default void setPivotDuty(double duty) {}

  /** Set inner roller motor duty cycle. */
  default void setInnerRollerDuty(double duty) {}

  /** Set outer roller motor duty cycle. */
  default void setOuterRollerDuty(double duty) {}

  /** Stop all intake motors. */
  default void stop() {}
}
