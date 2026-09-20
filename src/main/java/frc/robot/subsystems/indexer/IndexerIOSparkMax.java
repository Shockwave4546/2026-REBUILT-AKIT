// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerIOSparkMax implements IndexerIO {
  private final SparkMax motor;

  public IndexerIOSparkMax(int canId) {
    motor = new SparkMax(canId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(IndexerConstants.kIndexerMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IndexerConstants.kIndexerMotorCurrentLimit);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.motorVelocityRPM = motor.getEncoder().getVelocity();
    inputs.motorAppliedVolts = motor.getAppliedOutput() * 12.0;
    inputs.motorCurrentAmps = motor.getOutputCurrent();
    inputs.motorTempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setDuty(double duty) {
    motor.set(duty);
  }

  @Override
  public void stop() {
    motor.set(0.0);
  }
}
