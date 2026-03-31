// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IndexerIOInputs {
  public double motorVelocityRPM = 0.0;
  public double motorAppliedVolts = 0.0;
  public double motorCurrentAmps = 0.0;
  public double motorTempCelsius = 0.0;
}
