// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class LauncherIOInputs {
  // Feeder motor
  public double feederVelocityRPM = 0.0;
  public double feederAppliedVolts = 0.0;
  public double feederCurrentAmps = 0.0;
  public double feederTempCelsius = 0.0;

  // Shooter leader motor
  public double shooterLeaderVelocityRPM = 0.0;
  public double shooterLeaderAppliedVolts = 0.0;
  public double shooterLeaderCurrentAmps = 0.0;
  public double shooterLeaderTempCelsius = 0.0;

  // Shooter follower motor
  public double shooterFollowerVelocityRPM = 0.0;
  public double shooterFollowerAppliedVolts = 0.0;
  public double shooterFollowerCurrentAmps = 0.0;
  public double shooterFollowerTempCelsius = 0.0;
}
