// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  // Pivot and roller simulations
  private final FlywheelSim pivotSim;
  private final FlywheelSim innerRollerSim;
  private final FlywheelSim outerRollerSim;

  public IntakeIOSim() {
    // Pivot motor simulation (NEO motor)
    pivotSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.02, 1.0),
            DCMotor.getNEO(1),
            1.0);

    // Inner roller simulation (NEO 550)
    innerRollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.01, 1.0),
            DCMotor.getNeo550(1),
            1.0);

    // Outer roller simulation (NEO 550)
    outerRollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.01, 1.0),
            DCMotor.getNeo550(1),
            1.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update simulations
    pivotSim.update(LOOP_PERIOD_SECS);
    innerRollerSim.update(LOOP_PERIOD_SECS);
    outerRollerSim.update(LOOP_PERIOD_SECS);

    // Pivot outputs (convert rad/s to rot/s)
    inputs.pivotPositionRotations =
        pivotSim.getAngularVelocityRadPerSec()
            * LOOP_PERIOD_SECS
            / (2.0 * Math.PI); // Rough approximation for sim
    inputs.pivotVelocityRotPerSec = pivotSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.pivotAppliedVolts = pivotSim.getInputVoltage();
    inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.pivotTempCelsius = 25.0;

    // Inner roller outputs (RPM)
    inputs.innerRollerVelocityRPM =
        innerRollerSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    inputs.innerRollerAppliedVolts = innerRollerSim.getInputVoltage();
    inputs.innerRollerCurrentAmps = innerRollerSim.getCurrentDrawAmps();
    inputs.innerRollerTempCelsius = 25.0;

    // Outer roller outputs (RPM)
    inputs.outerRollerVelocityRPM =
        outerRollerSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    inputs.outerRollerAppliedVolts = outerRollerSim.getInputVoltage();
    inputs.outerRollerCurrentAmps = outerRollerSim.getCurrentDrawAmps();
    inputs.outerRollerTempCelsius = 25.0;
  }

  @Override
  public void setPivotDuty(double duty) {
    pivotSim.setInputVoltage(duty * 12.0);
  }

  @Override
  public void setInnerRollerDuty(double duty) {
    innerRollerSim.setInputVoltage(duty * 12.0);
  }

  @Override
  public void setOuterRollerDuty(double duty) {
    outerRollerSim.setInputVoltage(duty * 12.0);
  }

  @Override
  public void stop() {
    setPivotDuty(0.0);
    setInnerRollerDuty(0.0);
    setOuterRollerDuty(0.0);
  }
}
