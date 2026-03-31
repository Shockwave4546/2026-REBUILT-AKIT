// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakeIOInputs {
  // Pivot
  public double pivotPositionRotations = 0.0;
  public double pivotVelocityRotPerSec = 0.0;
  public double pivotAppliedVolts = 0.0;
  public double pivotCurrentAmps = 0.0;
  public double pivotTempCelsius = 0.0;

  // Inner Roller
  public double innerRollerVelocityRPM = 0.0;
  public double innerRollerAppliedVolts = 0.0;
  public double innerRollerCurrentAmps = 0.0;
  public double innerRollerTempCelsius = 0.0;

  // Outer Roller
  public double outerRollerVelocityRPM = 0.0;
  public double outerRollerAppliedVolts = 0.0;
  public double outerRollerCurrentAmps = 0.0;
  public double outerRollerTempCelsius = 0.0;
}
