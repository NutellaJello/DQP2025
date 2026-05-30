# RedClose15 Migration + Phase 2 Naming — Design Spec

**Date:** 2026-05-30
**Scope:** `RedClose15.java` migration to `BaseAuto`, plus the full Phase 2 naming sweep from `2026-05-03-java-refactor-design.md`
**Goal:** Make `RedClose15` a proper `BaseAuto` subclass (eliminating its ~200 lines of duplicated code), and complete all Phase 2 naming fixes across the entire codebase in the same pass.

No runtime behaviour changes except: `RedClose15` gains the servo initial-position fix and the `stop()` stopper-close fix automatically by inheriting `BaseAuto.initHardware()` and `BaseAuto.stop()`.

---

## Files Changed (15 total)

| File | Work |
|------|------|
| `subsystems/GoalPos.java` | Field + method renames |
| `subsystems/DecodeDriveTrain.java` | Field + method + local var renames |
| `Autos/BaseAuto.java` | Method rename, add idle-move overload, field renames |
| `Autos/RedClose15.java` | Full BaseAuto migration + Phase 2 renames |
| `Autos/BlueCloseGate.java` | Callsite updates + path field renames |
| `Autos/RedCloseGate.java` | Same |
| `Autos/BlueClose.java` | Same |
| `Autos/RedClose.java` | Same |
| `Autos/BlueFar.java` | Same |
| `Autos/RedFar.java` | Same |
| `Autos/BlueFarGate.java` | Same |
| `Autos/RedFarGate.java` | Same |
| `BlueTeleopWebcam.java` | Callsite updates + field renames |
| `RedTeleopWebcam.java` | Same |
| `testcode/FlywheelTesting.java` | Callsite updates + field renames |

---

## Section 1 — GoalPos.java

| Change | Old | New |
|--------|-----|-----|
| Field | `a` | `goalX` |
| Field | `b` | `goalY` |
| Field | `c` | `goalZ` |
| Field | `camAngle` | `camElevationAngleDeg` |
| Method | `findAngle()` | `findBearing()` |

**Callsites for `findBearing()` (was `findAngle()`):** 10 active + 1 commented-out  
`BlueCloseGate`, `RedCloseGate`, `BlueClose`, `RedClose`, `BlueFar`, `RedFar`, `BlueFarGate`, `RedFarGate`, `BlueTeleopWebcam`, `RedTeleopWebcam`, `FlywheelTesting` (commented)

---

## Section 2 — DecodeDriveTrain.java

| Change | Old | New |
|--------|-----|-----|
| Field | `FL` | `frontLeft` |
| Field | `FR` | `frontRight` |
| Field | `BL` | `backLeft` |
| Field | `BR` | `backRight` |
| Method (both overloads) | `Teleop(...)` | `drive(...)` |
| Local var | `PowerFL` | `powerFl` |
| Local var | `PowerFR` | `powerFr` |
| Local var | `PowerBL` | `powerBl` |
| Local var | `PowerBR` | `powerBr` |

**Callsites for `drive()` (was `Teleop()`):**  
`BlueTeleopWebcam:148`, `RedTeleopWebcam:152`, `FlywheelTesting:151`, internal overload call inside `DecodeDriveTrain`

---

## Section 3 — BaseAuto.java

### Method rename
`baseInit()` → `initHardware()`

**Callsites:** all 8 existing subclasses call this from their `init()` method.

### New overload
Add `move(PathChain path, Runnable onComplete, boolean idle)`:
```java
public void move(PathChain path, Runnable onComplete, boolean idle) {
    if (!moving) {
        if (idle) flyWheel1.setVelocity(1100);
        follower.followPath(path, true);
        moving = true;
    }
    if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
        moving = false;
        onComplete.run();
    }
}
```
`RedClose15` uses this with `idle=true` for PRELOAD, OUTTAKE1, OUTTAKEB, OUTTAKE3. It also calls `flyWheel2.setVelocity(1100)` inline before those `move()` calls (since `flyWheel2` is local to `RedClose15`).

### Field renames
`p`, `d`, `i`, `f` → `pidP`, `pidD`, `pidI`, `pidF`

---

## Section 4 — RedClose15.java (full migration)

### Structural changes
- `extends OpMode` → `extends BaseAuto`
- Remove all imports already covered by BaseAuto
- Remove duplicate field declarations: `intake`, `turret`, `flyWheel1`, `stopper`, `flap`, `aprilTag`, `visionPortal`, `follower`, `actionTimer`, `opmodeTimer`, `xPos`, `yPos`, `heading`, `turretPos`, `range`, `camRange`, `bearing`, `camOffsetX`, `startingAngle`, `flapPos`, `gainSet`, `moving`, `hasEst`, PIDF fields, `goal` (becomes `goalPos` from BaseAuto)
- Remove dead fields: `xEst`, `yEst` (declared but never used)
- Remove duplicate methods: `initWebcam()`, `cameraControls()`, `loop()`, `start()`, `toFWV()`

### Remaining unique fields
`flyWheel2`, `lowLimit`, `highLimit`, `elevation`, `PathState` enum, all path `Pose` constants, `PathChain` fields

### Abstract method implementations
```java
@Override protected double getPIDFP()       { return 400; }
@Override protected GoalPos createGoalPos() { return new GoalPos(147, 143, 15.5); }
@Override protected Pose getStartPose()     { return start; }
@Override protected double getFWVConstant() { return 1150; }
```

### init()
`initHardware()` must be called first — it constructs `fwPID` (via `getPIDFP()`), which `flyWheel2.setPIDFCoefficients` depends on.

```java
@Override
public void init() {
    pathState = PathState.PRELOAD;
    initHardware();  // constructs fwPID before flyWheel2 uses it
    flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
    flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
    flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    flyWheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID);
}
```

### stop()
```java
@Override
public void stop() {
    flyWheel2.setVelocity(0);
    super.stop();
}
```

### setPathState() helper
```java
protected void setPathState(PathState newState) {
    pathState = newState;
    actionTimer.resetTimer();
}
```

### statePathUpdate() — Runnable API
Switch cases rewritten from `move(Path, PathState.NEXT)` to `move(path, () -> setPathState(PathState.NEXT))`.

Cases that pre-spin flywheels (PRELOAD, OUTTAKE1, OUTTAKEB, OUTTAKE3):
```java
case OUTTAKE1:
    flyWheel2.setVelocity(1100);
    move(outtake1, () -> setPathState(PathState.SHOOT1), true);
    break;
```

OUTTAKE2 also gets `idle=true` — this was missing in the original (code review Finding #7) and is fixed here since we're rewriting every case:
```java
case OUTTAKE2:
    flyWheel2.setVelocity(1100);
    move(outtake2, () -> setPathState(PathState.SHOOT2), true);
    break;
```

Time-expiry block adds `flyWheel2.setVelocity(0)` alongside `flyWheel1.setVelocity(0)`.

### aiming()
- `goal.findRange(...)` → `goalPos.findRange(...)`
- `goal.update(...)` → `goalPos.update(...)`
- `goal.findAngle(...)` → `goalPos.findBearing(...)`

### Phase 2 path field renames
Per spec table — `Preload` → `preload`, `Intake11` → `intake11`, `Intake12` → `intake12`, `Outtake1` → `outtake1`, `Opengate` → `openGate`, `BigBack` → `bigBack`, `OuttakeB` → `outtakeB`, `Intake2` → `intake2`, `Outtake2` → `outtake2`, `Intake31` → `intake31`, `Intake32` → `intake32`, `Outtake3` → `outtake3`, `End` → `end`

---

## Section 5 — Existing 8 BaseAuto subclasses

For each of `BlueCloseGate`, `RedCloseGate`, `BlueClose`, `RedClose`, `BlueFar`, `RedFar`, `BlueFarGate`, `RedFarGate`:

1. `baseInit()` → `initHardware()` in `init()`
2. `goalPos.findAngle(...)` → `goalPos.findBearing(...)` in `aiming()`
3. PascalCase `PathChain` fields → camelCase per spec tables in `2026-05-03-java-refactor-design.md`

---

## Section 6 — BlueTeleopWebcam, RedTeleopWebcam, FlywheelTesting

| Change | Old | New |
|--------|-----|-----|
| Callsite | `goal.findAngle()` / `goalPos.findAngle()` | `findBearing()` |
| Callsite | `drivetrain.Teleop(...)` | `drivetrain.drive(...)` |
| Field | `p`, `d`, `i`, `f` | `pidP`, `pidD`, `pidI`, `pidF` |
| Field (`BlueTeleopWebcam`, `RedTeleopWebcam`) | `FWTarget` | `flywheelTarget` |
| Field (`BlueTeleopWebcam`, `RedTeleopWebcam`) | `FWV1`, `FWV2` | `flywheelVelocity1`, `flywheelVelocity2` |
| Field (`FlywheelTesting`) | `FW1Target` | `flywheelTarget` |
| Field (`FlywheelTesting`) | `FWV1`, `FWV2` | `flywheelVelocity1`, `flywheelVelocity2` |

---

## Success Criteria

1. `./gradlew assembleDebug` passes with zero errors
2. `RedClose15` extends `BaseAuto` and has no duplicate hardware init or shared utility methods
3. No method names starting uppercase in TeamCode (`Teleop`, `baseInit` gone)
4. No single-letter PIDF field names (`p`, `i`, `d`, `f`) in TeamCode
5. No PascalCase `PathChain` field declarations in any auto class
6. `findAngle` does not appear anywhere in TeamCode
