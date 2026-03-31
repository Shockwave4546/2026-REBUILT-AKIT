// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax pivotMotor;
  private final SparkMax innerRollerMotor;
  private final SparkMax outerRollerMotor;

  private final AbsoluteEncoder pivotEncoder;

  public IntakeIOSparkMax(int pivotCanId, int innerRollerCanId, int outerRollerCanId) {
    // --- Pivot motor ---
    pivotMotor = new SparkMax(pivotCanId, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
        .inverted(IntakeConstants.kIntakePivotMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.kIntakePivotCurrentLimit);
    pivotConfig
        .absoluteEncoder
        .inverted(IntakeConstants.kIntakePivotEncoderInverted)
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0 / 60.0) // rotations per second
        .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = pivotMotor.getAbsoluteEncoder();

    // --- Inner roller motor ---
    innerRollerMotor = new SparkMax(innerRollerCanId, MotorType.kBrushless);
    SparkMaxConfig innerRollerConfig = new SparkMaxConfig();
    innerRollerConfig
        .inverted(IntakeConstants.kIntakeInnerRollerInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kIntakeRollerCurrentLimit);
    innerRollerMotor.configure(
        innerRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Outer roller motor ---
    outerRollerMotor = new SparkMax(outerRollerCanId, MotorType.kBrushless);
    SparkMaxConfig outerRollerConfig = new SparkMaxConfig();
    outerRollerConfig
        .inverted(IntakeConstants.kIntakeOuterRollerInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kIntakeRollerCurrentLimit);
    outerRollerMotor.configure(
        outerRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotPositionRotations = pivotEncoder.getPosition();
    inputs.pivotVelocityRotPerSec = pivotEncoder.getVelocity();
    inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * 12.0;
    inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
    inputs.pivotTempCelsius = pivotMotor.getMotorTemperature();

    inputs.innerRollerVelocityRPM = innerRollerMotor.getEncoder().getVelocity();
    inputs.innerRollerAppliedVolts = innerRollerMotor.getAppliedOutput() * 12.0;
    inputs.innerRollerCurrentAmps = innerRollerMotor.getOutputCurrent();
    inputs.innerRollerTempCelsius = innerRollerMotor.getMotorTemperature();

    inputs.outerRollerVelocityRPM = outerRollerMotor.getEncoder().getVelocity();
    inputs.outerRollerAppliedVolts = outerRollerMotor.getAppliedOutput() * 12.0;
    inputs.outerRollerCurrentAmps = outerRollerMotor.getOutputCurrent();
    inputs.outerRollerTempCelsius = outerRollerMotor.getMotorTemperature();
  }

  @Override
  public void setPivotDuty(double duty) {
    pivotMotor.set(duty);
  }

  @Override
  public void setInnerRollerDuty(double duty) {
    innerRollerMotor.set(duty);
  }

  @Override
  public void setOuterRollerDuty(double duty) {
    outerRollerMotor.set(duty);
  }

  @Override
  public void stop() {
    pivotMotor.set(0.0);
    innerRollerMotor.set(0.0);
    outerRollerMotor.set(0.0);
  }
}
