# 2026 Tuning & Integration TODO

## Status: ✅ SETUP AND SHOOT COMMAND WORKING IN SIM

The `setupAndShoot` command is fully functional with all control loops executing and safety gates operational.

---

## 1. DRIVE SUBSYSTEM TUNING

### 1.1 Holonomic Position Controller Tuning
**File**: `VisionCommands.java` (lines 30-34)

Current gains used for `setupAndShoot`:
```java
ANGLE_KP = 3.0              // ← Tuned to 3.0 to reduce oscillation
ANGLE_KD = 0.5
ANGLE_MAX_VELOCITY = 8.0 rad/s
ANGLE_MAX_ACCELERATION = 20.0 rad/s²
ANGLE_TOLERANCE = 2° (0.0349 rad)
```

**Distance controller** (lines 428-429 in setupAndShoot):
```java
distanceController = new PIDController(1.0, 0.0, 0.0)  // ← Reduced from P=2.0
distanceController.setTolerance(0.05)  // 5cm
```

**TODO**:
- [ ] Test angle controller on actual robot at various headings (0°, 45°, 90°, 180°, etc.)
  - Watch for overshoot, oscillation, settling time
  - Adjust P and D based on real inertia
- [ ] Test distance controller at 2.0m, 2.5m, 3.0m, 3.5m, 4.0m
  - Verify no overshoot beyond tolerance
  - Check settling speed (should be <0.5 sec)
  - Adjust P gain if oscillation occurs
- [ ] Document final tuned gains in `DriveConstants.java` if different from test values

**Expected outcome**: Smooth rotation to hub, stable distance holding ±0.05m

---

### 1.2 Drive Module PID (Velocity Control)
**File**: `DriveConstants.java` (lines 66-74)

Current values:
```java
driveKp = 0.0
driveKd = 0.0
driveKs = 0.0
driveKv = 0.1
```

**TODO**:
- [ ] System identification on actual robot (characterize velocity response)
  - Use WPILib SysId tool or manual ramp test
  - Measure steady-state velocity vs applied voltage
  - Determine kS (static friction), kV (velocity), kA (acceleration)
- [ ] Run PathPlanner characterization on robot
  - Will update `driveSimKv`, `driveSimKs` values
- [ ] Compare sim vs real gains
  - Adjust `driveKp`, `driveKd` if velocity overshoots

**Current sim values** (lines 75-77):
```java
driveSimP = 0.05
driveSimD = 0.0
driveSimKv = 0.0789
```

**Expected outcome**: Velocity tracks commanded setpoint within 0.1 m/s

---

### 1.3 Turn Module PID (Rotation Control)
**File**: `DriveConstants.java` (lines 85-86)

Current values:
```java
turnKp = 2.0
turnKd = 0.0
```

Sim values (lines 87-88):
```java
turnSimP = 8.0
turnSimD = 0.0
```

**TODO**:
- [ ] Test module steering response on real robot
  - Verify fast settling without overshoot
  - Check for any mechanical friction/binding issues
- [ ] Compare real vs sim gains
  - Real robot likely needs higher P due to friction
  - Adjust D if ringing occurs
- [ ] Validate continuous input wrapping (-π to π) works smoothly at 0°/±180° boundaries

**Expected outcome**: Module steers to target angle in <100ms, no oscillation

---

## 2. LAUNCHER/SHOOTER TUNING

### 2.1 Shooter Velocity Control
**File**: `LauncherConstants.java` (lines 30-34)

Current PID/FF values:
```java
kP_Shooter = 0.001
kI_Shooter = 0.0
kD_Shooter = 0.0
kS_Shooter = 0.0
kV_Shooter = 0.00015
kA_Shooter = 0.0
```

Target RPM:
```java
kShooterTargetRpm = 4000.0
kShooterShortRpm = 3000.0    // For short shots
kShooterLongRpm = 4500.0     // For long shots
kRpmTolerance = 100.0        // Current tolerance band
```

**TODO**:
- [ ] System identification on shooter motor
  - Measure open-loop voltage vs RPM linearity
  - Determine kS (static voltage), kV (volts per RPM)
  - Determine kA (acceleration feedforward)
- [ ] Run characterization test
  - Ramp voltage from 0% to 100% in 2-3 seconds
  - Record RPM response curve
  - Fit to model: `voltage = kS * sign(velocity) + kV * velocity + kA * acceleration`
- [ ] Tune PID gains
  - Start with P=0.0001, no D
  - Increase P until reaching target RPM smoothly (<1 sec spinup)
  - Add D if overshoot occurs
  - Goal: ±50-100 RPM steady-state error at target
- [ ] Test at multiple RPM setpoints (2000, 2500, 3000, 3500, 4000, 4500)
  - Verify linearity across range
  - Adjust feedforward coefficients if nonlinear response

**Expected outcome**: Shooter reaches target RPM in <1 second, holds ±100 RPM

---

### 2.2 Shooter RPM Lookup Table (Distance → RPM mapping)
**File**: `ShootingConstants.java`

Current table (ESTIMATED/TEST VALUES):
```java
private static final double[][] RPM_LOOKUP_TABLE = {
    {2.0, 2000},
    {2.5, 2200},
    {3.0, 2400},
    {3.5, 2600},
    {4.0, 2800},
};
```

**TODO** - CRITICAL FOR AUTONOMOUS:
- [ ] **Shoot from 2.0m**: Record actual RPM needed for consistent scoring
- [ ] **Shoot from 2.5m**: Record actual RPM needed
- [ ] **Shoot from 3.0m**: Record actual RPM needed
- [ ] **Shoot from 3.5m**: Record actual RPM needed
- [ ] **Shoot from 4.0m**: Record actual RPM needed
- [ ] Characterize shooter performance
  - Build trajectory model if possible (angle varies with distance)
  - Account for game piece friction
  - Note any wheel wear effects on consistency
- [ ] Create production lookup table
  - Replace test values with measured data
  - Ensure linear interpolation between points is accurate
  - Document measurement conditions (battery voltage, temperature, etc.)
- [ ] Test interpolation accuracy
  - Verify 2.2m shoots correctly (interpolated between 2.0-2.5)
  - Verify 3.7m shoots correctly (interpolated between 3.5-4.0)

**Expected outcome**: Game pieces consistently score from all tested distances

---

### 2.3 Feeder/Indexer Tuning
**File**: `IndexerConstants.java` and `LauncherConstants.java`

Current feeder control:
```java
kFeederVoltage = 12.0  // Full voltage (open-loop)
```

**TODO**:
- [ ] Test feeder speed at different distances
  - Verify pieces feed smoothly without jamming
  - Check if full 12V is needed or if less voltage works
  - Note any timing issues (pieces arriving too fast/slow at shooter)
- [ ] Characterize indexer response
  - Measure time from indexer start to piece arriving at shooter
  - Verify consistent across multiple pieces
- [ ] Test indexer safety
  - Confirm indexer stops immediately when `stop()` called
  - Verify no double-feeding or stuck pieces
- [ ] Validate intake-to-indexer transition
  - Ensure pieces from intake feed smoothly into indexer
  - Check for any bottlenecks or misalignment

**Expected outcome**: Smooth, reliable feeding with zero jams during autonomous

---

## 3. VISION/ALIGNMENT TUNING

### 3.1 Hub Detection & Distance Measurement
**File**: `VisionIO.java`, `VisionIOPhotonVision.java`, `VisionIOLimelight.java`

**TODO**:
- [ ] Verify hub target detection on real court
  - Test with actual reflective hub target in game lighting
  - Confirm target detection works from 2-4m distance
  - Check false positive rate
- [ ] Validate distance measurement accuracy
  - Compare vision distance to odometry distance
  - Record error at each distance (2.0, 2.5, 3.0, 3.5, 4.0m)
  - Adjust camera calibration if >10cm error
- [ ] Test at various angles to hub
  - Verify detection works at 45°, 90°, 135°, 180° (all quadrants)
  - Check for any "blind spots" where hub isn't detected
- [ ] Characterize latency
  - Measure time from image capture to reported distance
  - Account for pipeline processing delays

**Expected outcome**: Distance accuracy ±5-10cm, zero missed detections during competition

---

### 3.2 Rotation/Heading Accuracy
**File**: `GyroIO.java`, `GyroIOPigeon2.java`, `GyroIONavX.java`

**TODO**:
- [ ] Validate gyro calibration on real robot
  - Place robot on flat surface, run self-calibration routine
  - Rotate 360° and verify final heading matches starting
- [ ] Test heading drift over time
  - Run for 2+ minutes without moving
  - Record drift rate (degrees per minute)
  - Acceptable: <0.5°/min
- [ ] Test rotation accuracy at small angles
  - Command 1° rotation, verify actual matches
  - Command 10° rotation, verify actual matches
  - Check for hysteresis or dead zones

**Expected outcome**: Heading accurate ±1°, minimal drift during match

---

## 4. INTAKE SUBSYSTEM TUNING

### 4.1 Intake Pivot Calibration
**File**: `IntakeConstants.java`

Current positions (in motor rotations):
```java
kIntakePivotMinPosition = 0.25    // deployed
kIntakePivotMaxPosition = 0.66    // retracted
kIntakePivotDeployedPosition = 0.29
kIntakePivotPartiallyDeployedPosition = 0.367
kIntakePivotWeightlessPosition = 0.58
kIntakePivotRetractedPosition = 0.63
```

**TODO**:
- [ ] Verify mechanical deployment positions
  - Confirm "deployed" position clears bumpers and can intake pieces
  - Confirm "retracted" position is within frame perimeter
  - Test intermediate positions match intended heights
- [ ] Characterize pivot motor response
  - Measure time to move from deployed→retracted
  - Check for smooth motion without binding
  - Verify no overshoot at target positions
- [ ] Test intake roller speed
  - Ensure pieces grab and feed smoothly
  - Verify no spillage at any speed
- [ ] Validate weightless position
  - Confirm no load on motor when holding at this angle
  - Useful for long holds without current draw

**Expected outcome**: Smooth, fast (<0.5 sec) deployment and retraction

---

### 4.2 Intake Roller Control
**File**: `IntakeConstants.java`, `Intake.java`

Current settings:
```java
kIntakeRollerMaxRunPosition = 0.5  // Won't spin rollers when fully retracted
```

**TODO**:
- [ ] Test roller speed at various RPMs
  - Find minimum speed needed to grab pieces
  - Find maximum speed without damage
- [ ] Test roller performance on game pieces
  - Verify consistent pickup success rate (>95%)
  - Check for any jamming or slippage
- [ ] Test roller behavior during scoring
  - Ensure pieces release smoothly into shooter
  - Verify no pieces stuck in rollers after auto

**Expected outcome**: 100% reliable intake during autonomous

---

## 5. AUTONOMOUS COMMAND INTEGRATION

### 5.1 Complete Auto Path Testing
**Files**: PathPlanner `.auto` files, `RobotContainer.java`

**TODO**:
- [ ] Test `SetupAndShoot.auto` (simple test)
  - Run for full 10 seconds
  - Verify all 3 safety conditions: angle ✓, distance ✓, RPM ✓
  - Confirm indexer runs and pieces score
- [ ] Test `ShootFromMiddle.auto` (path + shoot)
  - Drive to hub area + shoot
  - Verify accuracy of path following
  - Confirm setup and shoot executes after path
- [ ] Test `TwoShot.auto` (path + shoot + retreat + shoot)
  - More complex scenario
  - Verify command chaining works
  - Check odometry accuracy over longer paths

**Expected outcome**: Autonomous routines score consistently, all pieces reach target

---

### 5.2 Safety & Abort Testing
**TODO**:
- [ ] Test timeout behavior
  - Run command without meeting safety conditions
  - Verify command stops at 10-second mark
  - Confirm all subsystems stop (drive, shooter, indexer)
- [ ] Test safety gate lockout
  - Manually break angle condition (rotate away) → indexer should stop
  - Manually break distance condition (drive away) → indexer should stop
  - Manually stall shooter (block wheels) → indexer should stop
  - Verify immediate response (<50ms) to any condition failure
- [ ] Test manual abort (driver interruption)
  - Press abort button during setup and shoot
  - Verify clean shutdown of all motors
  - Check for no residual current draw

**Expected outcome**: Zero unsafe conditions during competition

---

## 6. REAL ROBOT PERFORMANCE BASELINE

### 6.1 Shooter Performance Characterization
**TODO**:
- [ ] Record shooter current draw vs RPM
  - Build power consumption model
  - Verify battery can sustain during match
- [ ] Test shooter consistency (game piece variability)
  - Shoot 10 pieces at same distance/RPM
  - Measure score location variance
  - Document accuracy vs game piece condition (new vs worn)
- [ ] Test shooter performance over battery voltage range
  - Test at 13V (fresh battery)
  - Test at 11V (mid-match)
  - Test at 9V (end of match)
  - Determine if RPM lookup needs compensation

**Expected outcome**: Consistent scoring across battery voltage range

---

### 6.2 Drive Performance Baseline
**TODO**:
- [ ] Measure acceleration/max speed in autonomous
  - Verify matches or exceeds practice robot specs
  - Check for any slipping or traction issues
- [ ] Test precision at stopping
  - Command drive to specific pose
  - Measure overshoot and settle time
- [ ] Test rotation precision
  - Spin to specific angle, measure error
  - Test at multiple angles to check for bias

**Expected outcome**: Drive meets or exceeds performance specifications

---

## 7. VISION ACCURACY VALIDATION

### 7.1 Distance Measurement Across Robot Position
**TODO**:
- [ ] Place robot at 5 known positions around hub
  - At each position, record vision distance vs true distance
  - Calculate systematic error (bias)
  - Calculate random error (noise)
- [ ] Test at all distances (2.0, 2.5, 3.0, 3.5, 4.0m)
  - Verify <±0.1m error at each distance
  - Note any distance-dependent bias
- [ ] Validate hub target detection reliability
  - Count successful detections / total frames
  - Measure latency (frame age at processing)

**Expected outcome**: Distance accuracy ±5cm, 99%+ detection rate

---

## 8. PRODUCTION READINESS CHECKLIST

### 8.1 Code Quality
**TODO**:
- [ ] Review all tuning constants
  - Confirm final values in all Constants.java files
  - Document source of each constant (measured, calculated, or empirical)
  - Add comments explaining acceptable ranges
- [ ] Remove debug/test logging
  - Keep only essential console output
  - Remove per-frame logging in production code
- [ ] Code review for safety
  - Verify all timeout protections in place
  - Confirm all motors stop in finallyDo() blocks
  - Check for any floating-point edge cases

### 8.2 Documentation
**TODO**:
- [ ] Update all Constants files with measured values
- [ ] Document RPM lookup table source and methodology
- [ ] Create tuning guide for next year's team
- [ ] Document any mechanical adjustments needed (pivot angles, etc.)

### 8.3 Match Preparation
**TODO**:
- [ ] Verify all motors/sensors functional (pre-match inspection)
- [ ] Run self-tests (gyro calibration, module alignment)
- [ ] Load latest firmware on all SparkMax/SparkFlex
- [ ] Verify command registration in `RobotContainer` is correct
- [ ] Test all three autonomous routines one final time

---

## Priority Order for Execution

1. **CRITICAL** (must do before competition):
   - [ ] Shooter RPM lookup table (Section 2.2) — determines if pieces score
   - [ ] Angle/distance PID tuning on real robot (Section 1.1) — determines setup accuracy
   - [ ] Safety gate testing (Section 5.2) — prevents unsafe conditions

2. **HIGH** (should do before competition):
   - [ ] Shooter velocity control tuning (Section 2.1)
   - [ ] Vision accuracy validation (Section 7.1)
   - [ ] Complete auto path testing (Section 5.1)
   - [ ] Drive performance baseline (Section 6.2)

3. **MEDIUM** (nice to have, improve performance):
   - [ ] Drive module velocity control (Section 1.2)
   - [ ] Turn module control (Section 1.3)
   - [ ] Intake subsystem tuning (Section 4.0)
   - [ ] Vision hub detection validation (Section 3.1)

4. **LOW** (polish, future improvement):
   - [ ] Gyro calibration & drift characterization (Section 3.2)
   - [ ] Production readiness checklist (Section 8.0)

---

## Notes

- **Sim vs Real Gap**: Current gains are tuned for simulation; real robot may require different values due to friction, inertia, and mechanical wear
- **Battery Effect**: Shooter performance changes with battery voltage; may need runtime compensation
- **Temperature Effect**: Motors perform differently when warm; ensure testing conditions are repeatable
- **Game Piece Variability**: Different game pieces have different mass/friction; lookup table should be robust to this

---

*Last Updated: April 6, 2026*
*Status: Command functional, tuning in progress*
