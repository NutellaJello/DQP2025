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
FLYWHEEL_1      = "FW1"
FLYWHEEL_2      = "FW2"
FLYWHEEL_LEGACY = "FW"   // used in BlueSideClose/Far, RedSideClose/Far, TestAuto (older wiring)
INTAKE          = "intake"
TURRET          = "turret"
STOPPER         = "stopper"
FLAP            = "flap"

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
TURRET_TICKS_PER_180_DEG   = 976      // hardware-specific: change if gear ratio changes
TURRET_STARTING_ANGLE_DEG  = 0.0
TURRET_WRAP_GUARD_DEG      = 30.0     // used by close-side autos and teleop
TURRET_WRAP_GUARD_FAR_DEG  = 200.0    // used by BlueFarGate/RedFarGate (wider wrap window)

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
SHOOT_TIMEOUT_MS          = 2800     // BlueCloseGate, RedCloseGate
SHOOT_TIMEOUT_FAST_MS     = 1600     // RedClose15 only (different route timing)
INTAKE_SETTLE_MS          = 50       // close-path settle (far paths use 1500 — leave as literal)
INTAKE_FAR_DWELL_MS       = 1500     // far-path dwell in BlueFarGate/RedFarGate

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
| `Autos/BlueCloseGate.java` | Replace `"FW2"`, `976`, `0.973`, flap positions, `APRILTAG_ID_BLUE` (id==20), `SHOOT_TIMEOUT_MS` (2800ms), `29.3`s cutoff, `TURRET_WRAP_GUARD_DEG` (30°) |
| `Autos/RedCloseGate.java` | Same as BlueCloseGate but `APRILTAG_ID_RED` (id==24) |
| `Autos/RedClose15.java` | Same as RedCloseGate but use `SHOOT_TIMEOUT_FAST_MS` (1600ms) not `SHOOT_TIMEOUT_MS` |
| `Autos/BlueFarGate.java` | Replace `"FW1"` (if present), servo positions, `APRILTAG_ID_BLUE` (id==20), `TURRET_WRAP_GUARD_FAR_DEG` (200°) — **not** `TURRET_WRAP_GUARD_DEG`, `INTAKE_FAR_DWELL_MS` (1500ms) |
| `Autos/RedFarGate.java` | Same as BlueFarGate but `APRILTAG_ID_RED` (id==24) |
| `Autos/BlueClose.java` | Servo positions, turret constants (disabled but in scope) |
| `Autos/RedClose.java` | Same |
| `Autos/BlueFar.java` | Servo positions, turret constants |
| `Autos/RedFar.java` | Same |
| `BlueTeleopWebcam.java` | Replace all literals; replace inline `toFWV` formula with `RobotConstants.toFlywheelVelocity()`; use `APRILTAG_ID_BLUE` (id==20); use `TURRET_WRAP_GUARD_DEG` (±180° wrap — different range than auto, keep separately) |
| `RedTeleopWebcam.java` | Same as BlueTeleopWebcam but `APRILTAG_ID_RED` (id==24) |
| `subsystems/DecodeDriveTrain.java` | Replace `"FL"/"FR"/"BL"/"BR"`, `1.2`, `0.8`, damp ratios |
| `subsystems/GoalPos.java` | Replace `20` cam angle, `18` height floor |
| `BlueSideClose.java`, `BlueSideFar.java`, `RedSideClose.java`, `RedSideFar.java` | Replace `"intake"`, `"turret"`, `"stopper"`, `"flap"`, `"FL"/"FR"/"BL"/"BR"`; use `HardwareNames.FLYWHEEL_LEGACY` for `"FW"` (not FW1/FW2 — these files use a single flywheel with a different device name) |
| `TestAuto.java` | Replace `"FL"/"FR"/"BL"/"BR"`, `"intake"`, `"Webcam 1"`; use `HardwareNames.FLYWHEEL_LEGACY` for `"FW"` |
| `testcode/FlywheelTesting.java` | Replace hardware strings (`"FW1"`, `"intake"`, `"turret"`, `"Webcam 1"`), PIDF literals; use `APRILTAG_ID_RED` for id==24 (tuned on red-side field setup) |

---

## Phase 2: Naming Cleanup

### Method renames

| File | Current name | New name | Reason | Callsites to update |
|------|-------------|---------|--------|---------------------|
| `subsystems/DecodeDriveTrain.java` | `Teleop(...)` (both overloads) | `drive(...)` | Method names must start lowercase; `drive` is a clearer verb | `BlueTeleopWebcam.java:152`, `RedTeleopWebcam.java:152`, `testcode/FlywheelTesting.java:151`, and internal overload call inside `DecodeDriveTrain.java:85` |
| `Autos/BaseAuto.java` | `baseInit()` | `initHardware()` | Verb-first; describes what it does, not what class it belongs to | All 8 auto subclasses call this from `init()`: `BlueCloseGate`, `RedCloseGate`, `RedClose15`, `BlueFarGate`, `RedFarGate`, `BlueClose`, `RedClose`, `BlueFar`, `RedFar` |
| `subsystems/GoalPos.java` | `findAngle()` | `findBearing()` | Distinguishes from elevation angle; `bearing` is the correct navigation term | `BlueCloseGate`, `RedCloseGate`, `RedClose15`, `BlueFarGate`, `RedFarGate`, `BlueClose`, `RedClose`, `BlueFar`, `RedFar`, `BlueTeleopWebcam`, `RedTeleopWebcam`, `testcode/FlywheelTesting` (12 callsites total) |

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
| `testcode/FlywheelTesting.java` | `p`, `d`, `i`, `f` | `pidP`, `pidD`, `pidI`, `pidF` | Single letters reserved for loop indices |
| `testcode/FlywheelTesting.java` | `FW1Target` | `flywheelTarget` | camelCase; removes jargon prefix |
| `testcode/FlywheelTesting.java` | `FWV1`, `FWV2` | `flywheelVelocity1`, `flywheelVelocity2` | Same |

### Local variable renames

| File | Method | Current | New |
|------|--------|---------|-----|
| `subsystems/DecodeDriveTrain.java` | `drive(...)` | `PowerFL`, `PowerFR`, `PowerBL`, `PowerBR` | `powerFl`, `powerFr`, `powerBl`, `powerBr` |

### Path variable renames (PascalCase → camelCase)

`PathState` enum values (`PRELOAD`, `INTAKE1`, etc.) are already correct `UPPER_SNAKE_CASE` and are **not changed**. Only the `PathChain` field declarations are renamed.

**`BlueCloseGate.java` and `RedCloseGate.java`** (same set):

| Current | New |
|---------|-----|
| `Preload` | `preload` |
| `Intake1` | `intake1` |
| `Opengate` | `openGate` |
| `Outtake1` | `outtake1` |
| `Intake21`, `Intake22` | `intake21`, `intake22` |
| `Outtake2` | `outtake2` |
| `Intake31`, `Intake32` | `intake31`, `intake32` |
| `Outtake3` | `outtake3` |
| `End` | `end` |

**`RedClose15.java`:**

| Current | New |
|---------|-----|
| `Preload` | `preload` |
| `Intake11`, `Intake12` | `intake11`, `intake12` |
| `Outtake1` | `outtake1` |
| `Opengate` | `openGate` |
| `BigBack` | `bigBack` |
| `OuttakeB` | `outtakeB` |
| `Intake2` | `intake2` |
| `Outtake2` | `outtake2` |
| `Intake31`, `Intake32` | `intake31`, `intake32` |
| `Outtake3` | `outtake3` |
| `End` | `end` |

**`BlueFarGate.java`:**

| Current | New |
|---------|-----|
| `Preload` | `preload` |
| `AlignIntake` | `alignIntake` |
| `Intake1` | `intake1` |
| `Outtake1` | `outtake1` |
| `AlignIntake2` | `alignIntake2` |
| `Togate` | `toGate` |
| `Outtake2` | `outtake2` |
| `End` | `end` |

**`RedFarGate.java`:**

| Current | New |
|---------|-----|
| `Preload` | `preload` |
| `AlignIntake` | `alignIntake` |
| `Intake1` | `intake1` |
| `Outtake1` | `outtake1` |
| `Intake21`, `Intake22` | `intake21`, `intake22` |
| `Outtake2` | `outtake2` |
| `End` | `end` |

---

---

## Phase 3: Package and Class Name Reorganisation

Phase 3 is independent of Phases 1 and 2 and can be executed after either completes. It involves file moves (package changes) and class renames. All `@TeleOp`/`@Autonomous` display names are unaffected — those come from annotation `name=` strings, not class names.

### Package renames (directory moves)

| Current package | New package | Reason |
|-----------------|-------------|--------|
| `Autos/` | `autos/` | Java convention: all-lowercase package names |
| `TeleOp/` | `teleop/` | Same; capital T and O are non-standard |
| `pedroPathing/` | `config/` | Named after a third-party library; contains only the team's configuration for it |

> **Note:** `pedroPathing/` merges into `config/` alongside `HardwareNames` and `RobotConstants` from Phase 1. The `config/` package becomes the single home for all robot configuration.

### File moves (class placement fixes)

| File | Current location | New location | Reason |
|------|-----------------|--------------|--------|
| `BlueTeleopWebcam.java` | root package | `teleop/` | Belongs with teleop code |
| `RedTeleopWebcam.java` | root package | `teleop/` | Same |
| `TestAuto.java` | root package | `testcode/` | Is a test/diagnostic op-mode |
| `centerstageTeleOp.java` | root package | `testcode/` | Legacy diagnostic; not current competition code |
| `DecodeTeleop.java` | `FtcRobotController/` root | `TeamCode/.../teamcode/` | Misplaced outside TeamCode module entirely |

### Class renames

| Current name | New name | Reason |
|-------------|---------|--------|
| `DecodeDriveTrain` | `MecanumDrive` | Team name ("Decode") in a class name is misleading and tells readers nothing about the class's function |
| `GoalPos` | `TargetTracker` | Reads as a coordinate struct; the class is a stateful EMA-filtered sensor-fusion tracker |
| `BlueTeleopWebcam` | `BlueTeleop` | "Webcam" describes one of many features; the class is the full competition teleop controller |
| `RedTeleopWebcam` | `RedTeleop` | Same |
| `pedroPathing/Constants` | `config/FollowerConfig` | `Constants` is a generic anti-pattern and the class also contains mutable objects and a factory method |
| `centerstageTeleOp` | `CenterStageTeleOp` | Fixes PascalCase violation |
| `RedClose15` | `RedCloseExtended` | `15` is an unexplained number that breaks the `{Color}{Distance}{Variant}` naming pattern used by every other auto; `Extended` communicates that this route has more states (BIGBACK, OUTTAKEB) than the standard Gate variant. **Confirm with team before renaming — if `15` refers to a specific scoring target (e.g., 15 elements), use `RedClose15Elem` instead.** |

### Callsites for class renames

Each class rename requires updating:
1. The class declaration (`public class X`)
2. The file name (`X.java`)
3. All `import` statements referencing the class
4. Any `new X(...)` instantiations
5. Any variable declarations of type `X`

| Renamed class | Known callers |
|--------------|---------------|
| `MecanumDrive` (was `DecodeDriveTrain`) | `BlueTeleopWebcam`/`RedTeleop`, `FlywheelTesting`, all `BlueSide*`/`RedSide*` legacy autos |
| `TargetTracker` (was `GoalPos`) | `BaseAuto`, `BlueTeleopWebcam`/`RedTeleop`, `FlywheelTesting`, all active `Autos/` subclasses |
| `FollowerConfig` (was `Constants`) | `BaseAuto` (`createAutoFollower()`), `BlueTeleopWebcam`/`RedTeleop` (`createTeleopFollower()`) |

### Cleanup: `TeleOp/` package after moves

After `BlueTeleop` and `RedTeleop` are moved into `teleop/`, the two empty stub files (`OfficialBlueTeleop.java`, `OfficialRedTeleop.java`) should be deleted — they are placeholder files with no content, annotations, or class bodies.

---

## What is explicitly out of scope

- **Method decomposition** (`drive()`, `runOpMode()`, `statePathUpdate()`) — separate future effort
- **Access modifier fixes** (`public` → `private/protected`) — separate future effort
- **Structural changes** (new base classes, interface extraction) — not needed
- **Pedro Pathing `Tuning.java`** — third-party library code; not touched

---

## Success criteria

After Phase 1:
1. `./gradlew assembleDebug` passes with zero errors
2. All hardware device-name strings appear exactly once in the codebase (in `HardwareNames`)
3. Magic number `976` (turret ticks) appears nowhere outside `RobotConstants`
4. The `toFlywheelVelocity` formula exists in exactly one place

After Phase 2:
5. No method names starting with an uppercase letter in TeamCode
6. No single-letter field names (`a`, `b`, `c`, `p`, `i`, `d`, `f`) in TeamCode
7. No PascalCase `PathChain` field declarations in any auto class
8. `./gradlew assembleDebug` passes with zero errors

After Phase 3:
9. No package directories with uppercase letters under `teamcode/`
10. No working competition code in the root `teamcode/` package (only imports/passes through)
11. `DecodeDriveTrain`, `GoalPos`, `BlueTeleopWebcam`, `RedTeleopWebcam`, `Constants` (pedroPathing) do not appear anywhere in the codebase
12. `OfficialBlueTeleop.java` and `OfficialRedTeleop.java` deleted
13. `./gradlew assembleDebug` passes with zero errors
