// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launcher;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class LauncherIOSparkMax implements LauncherIO {
  private final SparkMax feederMotor;
  private final SparkFlex shooterLeader;
  private final SparkFlex shooterFollower;

  private final RelativeEncoder shooterEncoder;

  public LauncherIOSparkMax(int feederCanId, int shooterLeaderCanId, int shooterFollowerCanId) {
    // --- Feeder motor (NEO 550 on SparkMax) ---
    feederMotor = new SparkMax(feederCanId, MotorType.kBrushless);
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig
        .inverted(LauncherConstants.kFeederMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(LauncherConstants.kFeederMotorCurrentLimit);
    feederMotor.configure(
        feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Shooter leader (Vortex on SparkFlex) ---
    shooterLeader = new SparkFlex(shooterLeaderCanId, MotorType.kBrushless);
    SparkFlexConfig shooterLeaderConfig = new SparkFlexConfig();
    shooterLeaderConfig
        .inverted(LauncherConstants.kShooterLeaderInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(LauncherConstants.kShooterMotorCurrentLimit);
    shooterLeader.configure(
        shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterEncoder = shooterLeader.getEncoder();

    // --- Shooter follower (Vortex on SparkFlex, follows leader inverted) ---
    shooterFollower = new SparkFlex(shooterFollowerCanId, MotorType.kBrushless);
    SparkFlexConfig shooterFollowerConfig = new SparkFlexConfig();
    shooterFollowerConfig
        .follow(shooterLeader, LauncherConstants.kShooterFollowerInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(LauncherConstants.kShooterMotorCurrentLimit);
    shooterFollower.configure(
        shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.feederVelocityRPM = feederMotor.getEncoder().getVelocity();
    inputs.feederAppliedVolts = feederMotor.getAppliedOutput() * 12.0;
    inputs.feederCurrentAmps = feederMotor.getOutputCurrent();
    inputs.feederTempCelsius = feederMotor.getMotorTemperature();

    inputs.shooterLeaderVelocityRPM = shooterEncoder.getVelocity();
    inputs.shooterLeaderAppliedVolts = shooterLeader.getAppliedOutput() * 12.0;
    inputs.shooterLeaderCurrentAmps = shooterLeader.getOutputCurrent();
    inputs.shooterLeaderTempCelsius = shooterLeader.getMotorTemperature();

    inputs.shooterFollowerVelocityRPM = shooterFollower.getEncoder().getVelocity();
    inputs.shooterFollowerAppliedVolts = shooterFollower.getAppliedOutput() * 12.0;
    inputs.shooterFollowerCurrentAmps = shooterFollower.getOutputCurrent();
    inputs.shooterFollowerTempCelsius = shooterFollower.getMotorTemperature();
  }

  @Override
  public void setFeederDuty(double duty) {
    feederMotor.set(duty);
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooterLeader.setVoltage(volts);
  }

  @Override
  public void stop() {
    feederMotor.set(0.0);
    shooterLeader.set(0.0);
    // follower tracks leader automatically via hardware follow
  }
}
