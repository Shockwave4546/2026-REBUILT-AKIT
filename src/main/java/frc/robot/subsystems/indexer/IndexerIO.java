// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

public interface IndexerIO {
  /** Updates the set of loggable inputs. */
  default void updateInputs(IndexerIOInputs inputs) {}

  /** Set indexer motor duty cycle. */
  default void setDuty(double duty) {}

  /** Stop the indexer motor. */
  default void stop() {}
}
