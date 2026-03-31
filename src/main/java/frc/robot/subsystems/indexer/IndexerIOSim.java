// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final FlywheelSim motorSim;

  public IndexerIOSim() {
    // Indexer motor simulation (NEO 550)
    motorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.01, 1.0),
            DCMotor.getNeo550(1),
            1.0);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    motorSim.update(LOOP_PERIOD_SECS);

    inputs.motorVelocityRPM = motorSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    inputs.motorAppliedVolts = motorSim.getInputVoltage();
    inputs.motorCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.motorTempCelsius = 25.0;
  }

  @Override
  public void setDuty(double duty) {
    motorSim.setInputVoltage(duty * 12.0);
  }

  @Override
  public void stop() {
    setDuty(0.0);
  }
}
