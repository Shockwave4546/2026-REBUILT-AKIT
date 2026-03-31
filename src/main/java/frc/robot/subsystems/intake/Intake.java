// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstantsProfiled;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem with RoboRIO-side profiled PID control for the pivot arm and open-loop
 * duty-cycle control for the rollers with automatic unjam detection.
 *
 * <p>The pivot arm uses a TrapezoidProfile to generate smooth, velocity-limited trajectories and a
 * ProfiledPIDController to track the profile. An ArmFeedforward term compensates for gravity and
 * friction. The combined output is sent directly to the motor as a duty cycle.
 *
 * <p>The roller motors are controlled with open-loop duty cycles and include automatic unjam logic
 * that detects high current + low RPM conditions and reverses the outer roller while the inner
 * roller continues forward.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  // Pivot control
  private final ProfiledPIDController pivotPID;
  private final ArmFeedforward pivotFF;
  private Double targetPosition = null;
  private boolean pivotEnabled = false;

  // Roller control
  private boolean rollerRunning = false;
  private boolean rollerReversing = false;

  // Unjam detection state
  private long stallDetectionStartTimeMs = 0;
  private boolean isUnjamReversing = false;
  private long unjamReverseStartTimeMs = 0;
  private boolean isPivotStalled = false;

  // Tunable roller speeds (for live testing)
  private double innerRollerSpeed = IntakeConstants.kIntakeInnerRollerForwardSpeed;
  private double outerRollerSpeedMultiplier = IntakeConstants.kIntakeOuterRollerForwardSpeed;

  public Intake(IntakeIO io) {
    this.io = io;

    // Initialize ProfiledPIDController
    pivotPID =
        new ProfiledPIDController(
            IntakeConstantsProfiled.kP,
            IntakeConstantsProfiled.kI,
            IntakeConstantsProfiled.kD,
            new TrapezoidProfile.Constraints(
                IntakeConstantsProfiled.kMaxVelocity, IntakeConstantsProfiled.kMaxAcceleration));
    pivotPID.setTolerance(IntakeConstantsProfiled.kTolerance);

    // Initialize ArmFeedforward
    pivotFF =
        new ArmFeedforward(
            IntakeConstantsProfiled.kS,
            IntakeConstantsProfiled.kG,
            IntakeConstantsProfiled.kV,
            IntakeConstantsProfiled.kA);

    // Seed dashboard with tuning controls
    SmartDashboard.putNumber(
        "Intake/Tuning/Inner Roller Speed", IntakeConstants.kIntakeInnerRollerForwardSpeed);
    SmartDashboard.putNumber(
        "Intake/Tuning/Outer Speed Multiplier", IntakeConstants.kIntakeOuterRollerForwardSpeed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Intake/Position", inputs.pivotPositionRotations);
    Logger.recordOutput("Intake/Velocity", inputs.pivotVelocityRotPerSec);

    // Read tunable roller speeds from dashboard
    innerRollerSpeed =
        SmartDashboard.getNumber(
            "Intake/Tuning/Inner Roller Speed", IntakeConstants.kIntakeInnerRollerForwardSpeed);
    outerRollerSpeedMultiplier =
        SmartDashboard.getNumber(
            "Intake/Tuning/Outer Speed Multiplier", IntakeConstants.kIntakeOuterRollerForwardSpeed);

    // Update pivot control
    updatePivotControl();

    // Update roller control with unjam detection
    updateRollerControl();

    // Publish telemetry
    SmartDashboard.putBoolean("Intake/Pivot Enabled", pivotEnabled);
    SmartDashboard.putBoolean("Intake/Roller Running", rollerRunning);
    SmartDashboard.putBoolean("Intake/Pivot Stalled", isPivotStalled);
    if (targetPosition != null) {
      SmartDashboard.putNumber("Intake/Target Position (rot)", targetPosition);
      SmartDashboard.putNumber(
          "Intake/Position Error (rot)", targetPosition - inputs.pivotPositionRotations);
    }
  }

  private void updatePivotControl() {
    if (pivotEnabled && targetPosition != null) {
      // Calculate PID correction
      double pidOutput = pivotPID.calculate(inputs.pivotPositionRotations, targetPosition);

      // Get the profile's current setpoint
      TrapezoidProfile.State setpoint = pivotPID.getSetpoint();

      // Convert encoder rotations -> physical angle in radians for ArmFeedforward
      double scale =
          IntakeConstantsProfiled.kGravityZeroPosition
              - IntakeConstantsProfiled.kGravityPeakPosition;
      double posRad =
          ((setpoint.position - IntakeConstantsProfiled.kGravityPeakPosition) / scale)
              * (Math.PI / 2.0);
      double velRadPerSec = (setpoint.velocity / scale) * (Math.PI / 2.0);
      double ffOutput = pivotFF.calculate(posRad, velRadPerSec);

      double output =
          MathUtil.clamp(
              pidOutput + ffOutput,
              -1.0, // kIntakePivotMinOutput
              1.0); // kIntakePivotMaxOutput

      // Check for stall condition during deployment
      boolean isDeploying = setpoint.position < IntakeConstantsProfiled.kGravityZeroPosition - 0.2;
      boolean isMovingDown = inputs.pivotPositionRotations > setpoint.position;
      boolean isHighCurrent = inputs.pivotCurrentAmps > IntakeConstants.kPivotStallCurrentLimit;

      if (isDeploying && isMovingDown && isHighCurrent) {
        // Prevent gear skipping
        isPivotStalled = true;
        output = 0.0;
      }

      io.setPivotDuty(output);

      // Debug telemetry
      SmartDashboard.putNumber("Intake/PID Output", pidOutput);
      SmartDashboard.putNumber("Intake/FF Output", ffOutput);
      SmartDashboard.putNumber("Intake/Profile Position (rot)", setpoint.position);
      SmartDashboard.putNumber("Intake/Profile Velocity (rot/s)", setpoint.velocity);
    } else {
      io.setPivotDuty(0.0);
    }

    SmartDashboard.putNumber("Intake/Actual Position (rot)", inputs.pivotPositionRotations);
    SmartDashboard.putNumber("Intake/Actual Velocity (rot/s)", inputs.pivotVelocityRotPerSec);
  }

  private void updateRollerControl() {
    long nowMs = System.currentTimeMillis();
    boolean rollersAllowed =
        inputs.pivotPositionRotations < IntakeConstants.kIntakeRollerMaxRunPosition;

    // Check if unjam reverse period has elapsed
    if (isUnjamReversing) {
      long elapsedMs = nowMs - unjamReverseStartTimeMs;
      if (elapsedMs >= (IntakeConstants.kUnjamReverseTimeS * 1000)) {
        isUnjamReversing = false;
        stallDetectionStartTimeMs = 0;
      } else {
        // Outer reverses to clear jam; inner keeps pushing forward
        io.setInnerRollerDuty(IntakeConstants.kIntakeUnjamInnerSpeed);
        io.setOuterRollerDuty(IntakeConstants.kIntakeUnjamOuterSpeed);
        SmartDashboard.putString("Intake/Unjam Status", "REVERSING");
        return;
      }
    }

    // Detect jam condition: high current for sustained duration
    boolean isHighCurrent = inputs.outerRollerCurrentAmps > IntakeConstants.kUnjamCurrentThreshold;

    if (rollerRunning && rollersAllowed && isHighCurrent) {
      if (stallDetectionStartTimeMs == 0) {
        stallDetectionStartTimeMs = nowMs;
      }
      long stallDurationMs = nowMs - stallDetectionStartTimeMs;

      if (stallDurationMs >= (IntakeConstants.kUnjamDetectionTimeS * 1000)) {
        // Jam detected - initiate unjam
        isUnjamReversing = true;
        unjamReverseStartTimeMs = nowMs;
        stallDetectionStartTimeMs = 0;
        SmartDashboard.putString("Intake/Unjam Status", "JAM DETECTED");
      }
    } else {
      stallDetectionStartTimeMs = 0;
    }

    // Normal roller control
    if (rollerReversing && rollersAllowed) {
      io.setInnerRollerDuty(-innerRollerSpeed);
      io.setOuterRollerDuty(-innerRollerSpeed * outerRollerSpeedMultiplier);
      SmartDashboard.putString("Intake/Unjam Status", "REVERSING_MANUAL");
    } else if (rollerRunning && rollersAllowed) {
      io.setInnerRollerDuty(innerRollerSpeed);
      io.setOuterRollerDuty(innerRollerSpeed * outerRollerSpeedMultiplier);
      SmartDashboard.putString("Intake/Unjam Status", "RUNNING");
    } else {
      io.setInnerRollerDuty(0.0);
      io.setOuterRollerDuty(0.0);
      SmartDashboard.putString("Intake/Unjam Status", "STOPPED");
    }

    // Roller diagnostics
    SmartDashboard.putNumber("Intake/Inner Roller RPM", inputs.innerRollerVelocityRPM);
    SmartDashboard.putNumber("Intake/Inner Roller Current (A)", inputs.innerRollerCurrentAmps);
    SmartDashboard.putNumber("Intake/Outer Roller RPM", inputs.outerRollerVelocityRPM);
    SmartDashboard.putNumber("Intake/Outer Roller Current (A)", inputs.outerRollerCurrentAmps);
    SmartDashboard.putBoolean("Intake/High Current", isHighCurrent);
  }

  /** Set the target pivot position. */
  public void setTargetPosition(double positionRotations) {
    double clamped =
        MathUtil.clamp(
            positionRotations,
            IntakeConstants.kIntakePivotMinPosition,
            IntakeConstants.kIntakePivotMaxPosition);

    // Seed the profile from current state only when coming out of idle
    if (!pivotEnabled) {
      pivotPID.reset(inputs.pivotPositionRotations);
    }

    targetPosition = clamped;
    pivotEnabled = true;
    isPivotStalled = false;
  }

  /** Stop all motion and disable the pivot. */
  public void stop() {
    pivotEnabled = false;
    targetPosition = null;
    rollerRunning = false;
    rollerReversing = false;
    io.stop();
  }

  /** Stop only the pivot without affecting rollers. */
  public void stopPivot() {
    pivotEnabled = false;
    targetPosition = null;
    io.setPivotDuty(0.0);
  }

  /** Enable the intake rollers. */
  public void run() {
    rollerRunning = true;
    rollerReversing = false;
  }

  /** Stop the intake rollers. */
  public void stopRollers() {
    rollerRunning = false;
    rollerReversing = false;
  }

  /** Reverse the intake rollers. */
  public void runReverse() {
    rollerRunning = false;
    rollerReversing = true;
  }

  // Query methods
  @AutoLogOutput(key = "Intake/Position")
  public double getPosition() {
    return inputs.pivotPositionRotations;
  }

  @AutoLogOutput(key = "Intake/At Target")
  public boolean isAtTarget() {
    return pivotEnabled && targetPosition != null && pivotPID.atSetpoint();
  }

  @AutoLogOutput(key = "Intake/Deployed")
  public boolean isDeployed() {
    return inputs.pivotPositionRotations < IntakeConstants.kIntakePivotDeployedThreshold;
  }

  @AutoLogOutput(key = "Intake/Retracted")
  public boolean isRetracted() {
    return inputs.pivotPositionRotations > IntakeConstants.kIntakePivotRetractedThreshold;
  }

  public boolean isRollerRunning() {
    return rollerRunning;
  }
}
