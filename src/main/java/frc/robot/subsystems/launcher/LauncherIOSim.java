// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class LauncherIOSim implements LauncherIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final FlywheelSim feederSim;
  private final FlywheelSim shooterSim;

  public LauncherIOSim() {
    // Feeder motor simulation (NEO 550)
    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.01, 1.0),
            DCMotor.getNeo550(1),
            1.0);

    // Shooter simulation (simulated as dual NEO motors)
    shooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.05, 1.0),
            DCMotor.getNEO(2),
            1.0);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    feederSim.update(LOOP_PERIOD_SECS);
    shooterSim.update(LOOP_PERIOD_SECS);

    // Feeder outputs
    inputs.feederVelocityRPM = feederSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    inputs.feederAppliedVolts = feederSim.getInputVoltage();
    inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();
    inputs.feederTempCelsius = 25.0;

    // Shooter outputs (both leader and follower are the same in sim)
    double shooterRPM = shooterSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    inputs.shooterLeaderVelocityRPM = shooterRPM;
    inputs.shooterLeaderAppliedVolts = shooterSim.getInputVoltage();
    inputs.shooterLeaderCurrentAmps = shooterSim.getCurrentDrawAmps();
    inputs.shooterLeaderTempCelsius = 25.0;

    inputs.shooterFollowerVelocityRPM = shooterRPM;
    inputs.shooterFollowerAppliedVolts = shooterSim.getInputVoltage();
    inputs.shooterFollowerCurrentAmps = shooterSim.getCurrentDrawAmps();
    inputs.shooterFollowerTempCelsius = 25.0;
  }

  @Override
  public void setFeederDuty(double duty) {
    feederSim.setInputVoltage(duty * 12.0);
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooterSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setFeederDuty(0.0);
    setShooterVoltage(0.0);
  }
}
