# Live PID Tuning Guide

This branch (`tuning/live-pid`) contains tools for real-time PID tuning without code recompilation. Perfect for characterizing and optimizing your swerve drive on the actual robot!

## Overview

Three main components:

1. **PIDTuner.java** - Utility class for updating PID gains from SmartDashboard
2. **AngleTuningCommand.java** - Test command for tuning rotation/steering PID
3. **DriveVelocityTuningCommand.java** - Test command for tuning drive velocity PID

## Setup

### 1. Bind Tuning Commands to a Button (Optional)

In `RobotContainer.java`, add:

```java
// For angle/rotation tuning
driverController.a().onTrue(new AngleTuningCommand(drive));

// For drive velocity tuning
driverController.b().onTrue(new DriveVelocityTuningCommand(drive));
```

Or access via Dashboard:

```java
SmartDashboard.putData("Commands/Angle Tuner", new AngleTuningCommand(drive));
SmartDashboard.putData("Commands/Drive Velocity Tuner", new DriveVelocityTuningCommand(drive));
```

### 2. Deploy to Robot

```bash
./gradlew deploy
```

## Usage Guide

### **Angle/Rotation Tuning**

Use `AngleTuningCommand` to tune steering module rotation control.

**Step 1: Start the Command**
- Press the bound button, or click "Angle Tuner" in SmartDashboard
- You'll see: `========== ANGLE TUNING STARTED ==========`

**Step 2: Open SmartDashboard**
- Tab: `AngleTuner`
- You'll see controls:
  - `Target Angle (deg)` - Set desired heading (0-360°)
  - `kP` - Proportional gain (start: 2.0)
  - `kI` - Integral gain (leave at 0.0)
  - `kD` - Derivative gain (start: 0.0)

**Step 3: Start Tuning**

1. Set `Target Angle (deg)` to 90°
2. The robot will rotate to face that angle
3. Watch `Position Error (deg)` in SmartDashboard:
   - Should smoothly decrease to near 0
   - Should NOT overshoot past 0 (oscillate)

4. Adjust gains:
   - **Error converges slowly?** → Increase `kP`
   - **Robot oscillates around target?** → Decrease `kP` or increase `kD`
   - **Can't reach target smoothly?** → Tweak `kD` (0.0-0.5 typical range)

5. Test at multiple angles:
   - Try 0°, 45°, 90°, 180°, 270°
   - Should behave consistently at all angles

**Step 4: Record Gains**
Once you find good values, update `DriveConstants.java`:
```java
public static final double turnKp = 2.5;  // Your tuned value
public static final double turnKd = 0.1;  // Your tuned value
```

### **Drive Velocity Tuning**

Use `DriveVelocityTuningCommand` to tune forward/backward velocity control.

**Step 1: Start the Command**
- Press button or click "Drive Velocity Tuner"
- Ensure you have 10+ meters of clear space to drive

**Step 2: Open SmartDashboard**
- Tab: `DriveVelTuner`
- Controls:
  - `Target Velocity (m/s)` - Desired speed (start: 0.5)
  - `kP` - Proportional gain (start: 0.0)
  - `kI` - Integral gain (leave at 0.0)
  - `kD` - Derivative gain (start: 0.0)
  - `kV` - Feedforward gain (use from DriveConstants: 0.1)

**Step 3: Start Tuning**

1. Set `Target Velocity (m/s)` to 0.5 (conservative start)
2. Robot accelerates forward at that speed
3. Watch:
   - `Velocity Error (m/s)` - Should stay small (<0.1)
   - `Total Output (V)` - Should stabilize around 3-5V at 0.5 m/s

4. Tune PID:
   - **Takes too long to reach speed?** → Increase `kP` (try 0.001 → 0.002)
   - **Overshoots then backs up?** → Decrease `kP`
   - **Steady oscillation?** → Add small `kD` (try 0.0001)

5. Test at higher speeds:
   - Try 1.0, 2.0, 3.0 m/s
   - Should maintain velocity smoothly

**Step 4: Record Gains**
Update `DriveConstants.java`:
```java
public static final double driveKp = 0.001;  // Your tuned value
public static final double driveKd = 0.0;    // Your tuned value
```

## Console Output

Both commands print periodic updates (every ~1 second):

**Angle Tuning:**
```
[ANGLE_TUNER] Target: 90.0° | Current: 45.2° | Error: 44.8° | Output: 1.50 rad/s | P:2.0 D:0.1
```

**Drive Velocity:**
```
[DRIVE_VEL_TUNER] Target: 0.50 m/s | Est: 0.48 m/s | Error: 0.02 m/s | FF: 0.05 V | PID: 0.01 V | P:0.001 D:0.0
```

## Tips & Tricks

### General PID Tuning Philosophy
1. **Start with P only** (I=0, D=0)
   - Increase P until robot reaches target but oscillates
   - Back off P to prevent oscillation

2. **Add D to reduce oscillation**
   - Small amounts of D (~5-10% of P value) dampen overshooting
   - Too much D makes response sluggish

3. **Rarely need I for these applications**
   - Leave I at 0.0 for simpler tuning

### SmartDashboard Tips
- **Live plots**: Right-click "Position Error" → "Plot"
- **Compare runs**: Open multiple instances to record baseline
- **Tune while running**: Changes apply immediately, no redeploy!

### Real Robot vs Simulation
- Real robot typically needs **higher P gains** (more friction to overcome)
- Real robot may need **more D** for damping (mechanical slop/backlash)
- Sim values are often 50-70% of real robot values

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot won't rotate (angle stays at 0) | Increase `kP` significantly (try 5.0) |
| Robot oscillates wildly | Decrease `kP` by 50%, increase `kD` |
| Slow to reach target angle | Increase `kP`, may need `kD` for stability |
| Drive won't accelerate forward | Check if `Target Velocity` is set (not 0) |
| Velocity overshoots then backs up | Decrease `kP` |
| Can't reach steady velocity | Check robot isn't stalled (try 0.5 m/s not 4.0) |

## Workflow Example

```
1. Deploy code
2. Place robot on field with space to move
3. Start AngleTuningCommand
4. Adjust Target Angle to 45°
5. Start with kP=2.0, kD=0.0
6. Observe: Robot slowly rotates with large overshoot
7. Increase kP to 4.0
8. Observe: Faster rotation, some oscillation
9. Add kD=0.2
10. Observe: Smooth rotation, minimal overshoot
11. Save: Update DriveConstants.java with kP=4.0, kD=0.2
12. Redeploy and disable tuning commands (remove from RobotContainer)
```

## Moving to Production

Once tuning is complete:

1. **Save your gains** in `DriveConstants.java`
2. **Switch back to main branch**: `git checkout main`
3. **Cherry-pick your constant changes** (don't merge tuning commands)
4. **Remove tuning commands** from `RobotContainer.java`
5. **Deploy final code** to robot

```bash
# Save constants to a text file for reference
git diff tuning/live-pid DriveConstants.java > drive-constants-tuning.txt

# Switch to main
git checkout main

# Manually update DriveConstants.java with your values
# Then redeploy
./gradlew deploy
```

## Files Modified

- `PIDTuner.java` - New utility class
- `AngleTuningCommand.java` - New command for angle tuning
- `DriveVelocityTuningCommand.java` - New command for velocity tuning
- `RobotContainer.java` - (Optional) Add button bindings

## Questions?

Common issues when tuning:
- Gyro drift? Make sure gyro is calibrated before tuning
- Motors not responding? Check CAN IDs and motor inversions in constants
- Unstable response? Try reducing target velocity/angle to test signal path

---

*Last Updated: May 2, 2026*
Happy tuning! 🔧
