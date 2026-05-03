# Java Best-Practices Refactor — Design Spec

**Date:** 2026-05-03  
**Scope:** All TeamCode files (active + disabled)  
**Goal:** Eliminate magic numbers, scattered hardware-name strings, and naming-convention violations across the codebase without changing runtime behaviour.

---

## Summary

A two-phase refactor:

1. **Phase 1 — Constants extraction:** Create `HardwareNames` and `RobotConstants` config classes, then sweep all files replacing hard-coded string literals and magic numbers with references to those constants. Extract the duplicated flywheel-velocity formula into a shared static method.

2. **Phase 2 — Naming cleanup:** Rename fields, methods, and local variables that violate Java conventions (camelCase methods/fields, UPPER_SNAKE_CASE constants, PascalCase only for types).

No runtime behaviour changes in either phase. No method decomposition (that is a separate future effort).

---

## Phase 1: Constants Extraction

### New files

#### `config/HardwareNames.java`

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/HardwareNames.java
```

All hardware device-name strings as `public static final String`:

```java
// Drivetrain
FRONT_LEFT  = "FL"
FRONT_RIGHT = "FR"
BACK_LEFT   = "BL"
BACK_RIGHT  = "BR"

// Mechanisms
FLYWHEEL_1 = "FW1"
FLYWHEEL_2 = "FW2"
INTAKE     = "intake"
TURRET     = "turret"
STOPPER    = "stopper"
FLAP       = "flap"

// Sensors
WEBCAM     = "Webcam 1"
PINPOINT   = "pinpoint"
```

#### `config/RobotConstants.java`

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConstants.java
```

All magic numbers as `public static final double` (or `int` where appropriate), grouped by subsystem:

```java
// --- Turret ---
TURRET_TICKS_PER_180_DEG  = 976      // hardware-specific: change if gear ratio changes
TURRET_STARTING_ANGLE_DEG = 0.0
TURRET_WRAP_GUARD_DEG     = 30.0

// --- PIDF defaults (shared across autos and teleop) ---
PIDF_F   = 13.5
PIDF_I   = 0.0
PIDF_D   = 0.0

// --- Servo positions ---
STOPPER_CLOSED         = 0.9
STOPPER_OPEN           = 0.973
FLAP_POSITION_CLOSE    = 0.0
FLAP_POSITION_MID      = 0.2
FLAP_POSITION_FAR      = 0.24

// --- Camera ---
CAMERA_WARMUP_MS       = 500
CAMERA_EXPOSURE_MS     = 2
CAMERA_GAIN            = 100
CAM_ELEVATION_ANGLE_DEG = 20.0

// --- Flywheel velocity formula coefficients ---
// Used in: BaseAuto.toFWV(), BlueTeleopWebcam.firing()
// Formula: COEFF_A * range^2 + COEFF_B * range + constant
FLYWHEEL_VELOCITY_COEFF_A = 0.00673
FLYWHEEL_VELOCITY_COEFF_B = 5.54

// --- Drivetrain ---
STRAFE_CORRECTION = 1.2     // lateral correction factor for mecanum
TURN_SCALE        = 0.8     // gamepad right-stick turn scale
DAMP_SPEED_SLOW   = 0.35    // speed ratio when slow-mode active
DAMP_TURN_SLOW    = -0.3    // turn ratio when slow-mode active
TURN_RATIO_NORMAL = -0.6    // turn ratio in normal mode

// --- AprilTag IDs ---
APRILTAG_ID_BLUE = 20
APRILTAG_ID_RED  = 24

// --- Timing ---
AUTO_SHOOT_CUTOFF_SECONDS = 29.3
SHOOT_TIMEOUT_MS          = 2800
INTAKE_SETTLE_MS          = 50

// --- Misc ---
POSE_EPSILON = 1e-8         // prevents zero-length path in Pedro Pathing hold
MIN_GOAL_HEIGHT_INCHES = 18.0
```

#### Shared utility method on `RobotConstants`

```java
public static double toFlywheelVelocity(double range, double constant) {
    return (FLYWHEEL_VELOCITY_COEFF_A * range * range)
         + (FLYWHEEL_VELOCITY_COEFF_B * range)
         + constant;
}
```

This replaces:
- `BaseAuto.toFWV()` — which is refactored to delegate here
- Inline formula in `BlueTeleopWebcam.firing()`

### Files modified in Phase 1

| File | What changes |
|------|-------------|
| `Autos/BaseAuto.java` | Replace `"FW1"`, `"turret"`, `"stopper"`, `"flap"`, `"Webcam 1"`, `0.9`/`0.2` servo init, `500`ms guard, `2`ms exposure, `100` gain, `13.5`/`0` PIDF defaults; delegate `toFWV()` to `RobotConstants.toFlywheelVelocity()` |
| `Autos/BlueCloseGate.java` | Replace `"FW2"`, `976`, `0.973`, flap positions, `20` AprilTag ID, `2800`ms timeout, `29.3`s cutoff, `30` wrap guard |
| `Autos/RedCloseGate.java` | Same as BlueCloseGate |
| `Autos/RedClose15.java` | Same as BlueCloseGate |
| `Autos/BlueFarGate.java` | Turret constants, servo positions |
| `Autos/RedFarGate.java` | Turret constants, servo positions |
| `Autos/BlueClose.java` | Servo positions, turret constants (disabled but in scope) |
| `Autos/RedClose.java` | Same |
| `Autos/BlueFar.java` | Servo positions, turret constants |
| `Autos/RedFar.java` | Same |
| `BlueTeleopWebcam.java` | Replace all literals; replace inline `toFWV` formula with `RobotConstants.toFlywheelVelocity()` |
| `RedTeleopWebcam.java` | Same |
| `subsystems/DecodeDriveTrain.java` | Replace `"FL"/"FR"/"BL"/"BR"`, `1.2`, `0.8`, damp ratios |
| `subsystems/GoalPos.java` | Replace `20` cam angle, `18` height floor |
| `BlueSideClose.java`, `BlueSideFar.java`, `RedSideClose.java`, `RedSideFar.java` | Hardware name strings wherever present |
| `testcode/FlywheelTesting.java` | Hardware strings, PIDF literals |

---

## Phase 2: Naming Cleanup

### Method renames

| File | Current name | New name | Reason |
|------|-------------|---------|--------|
| `subsystems/DecodeDriveTrain.java` | `Teleop(...)` | `drive(...)` | Method names must start lowercase; `drive` is a clearer verb |
| `Autos/BaseAuto.java` | `baseInit()` | `initHardware()` | Verb-first; describes what it does, not what class it belongs to |
| `subsystems/GoalPos.java` | `findAngle()` | `findBearing()` | Distinguishes from elevation angle; `bearing` is the correct navigation term |

### Field renames

| File | Current | New | Reason |
|------|---------|-----|--------|
| `subsystems/GoalPos.java` | `a`, `b`, `c` | `goalX`, `goalY`, `goalZ` | Single letters have no semantic meaning |
| `subsystems/GoalPos.java` | `camAngle` | `camElevationAngleDeg` | Descriptive + includes unit |
| `Autos/BaseAuto.java` | `p`, `d`, `i`, `f` | `pidP`, `pidD`, `pidI`, `pidF` | Single letters reserved for loop indices |
| `BlueTeleopWebcam.java` | `p`, `d`, `i`, `f` | `pidP`, `pidD`, `pidI`, `pidF` | Same |
| `BlueTeleopWebcam.java` | `FWTarget` | `flywheelTarget` | camelCase; removes jargon prefix |
| `BlueTeleopWebcam.java` | `FWV1`, `FWV2` | `flywheelVelocity1`, `flywheelVelocity2` | Same |
| `RedTeleopWebcam.java` | Same as above | Same as above | Mirrored file |
| `subsystems/DecodeDriveTrain.java` | `FL`, `FR`, `BL`, `BR` | `frontLeft`, `frontRight`, `backLeft`, `backRight` | ALL_CAPS reserved for constants |

### Local variable renames

| File | Method | Current | New |
|------|--------|---------|-----|
| `subsystems/DecodeDriveTrain.java` | `drive(...)` | `PowerFL`, `PowerFR`, `PowerBL`, `PowerBR` | `powerFl`, `powerFr`, `powerBl`, `powerBr` |

### Path variable renames (PascalCase → camelCase)

Affects all active autos in `Autos/` that declare `PathChain` fields with PascalCase names. For example in `BlueCloseGate`:

| Current | New |
|---------|-----|
| `Preload` | `preload` |
| `Intake1` | `intake1` |
| `Opengate` | `openGate` |
| `Outtake1` | `outtake1` |
| `End` | `end` |

The `PathState` enum values (`PRELOAD`, `INTAKE1`, etc.) are already correct `UPPER_SNAKE_CASE` and are **not changed**.

---

## What is explicitly out of scope

- **Method decomposition** (`Teleop()`, `runOpMode()`, `statePathUpdate()`) — separate future effort
- **Access modifier fixes** (`public` → `private/protected`) — separate future effort
- **Structural changes** (new base classes, interface extraction) — not needed
- **Pedro Pathing `Tuning.java`** — third-party library code; not touched
- **`centerstageTeleOp.java`** — legacy from a previous game season; skipped to avoid noise

---

## Success criteria

1. `./gradlew assembleDebug` passes with zero errors after each phase
2. All hardware device-name strings appear exactly once in the codebase (in `HardwareNames`)
3. Magic number `976` (turret ticks) appears nowhere outside `RobotConstants`
4. No method names starting with an uppercase letter in TeamCode
5. No single-letter field names (`a`, `b`, `c`, `p`, `i`, `d`, `f`) in TeamCode
6. The `toFlywheelVelocity` formula exists in exactly one place
