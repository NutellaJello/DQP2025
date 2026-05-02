# DQP2025 Code Review — Main Branch
**Date:** 2026-05-02

---

## Part 1 — Build-Breaking Bugs

### 1. `Constants.createFollower()` does not exist
**Files:** `RedClose15.java:192`, `FlywheelTesting.java:106`

Both call `Constants.createFollower(hardwareMap)` but `Constants.java` only defines `createAutoFollower()` and `createTeleopFollower()`. Per the Pedro Pathing docs, `createFollower` is the canonical method name.

**Fix:** Add to `Constants.java`:
```java
public static Follower createFollower(HardwareMap hardwareMap) {
    return createAutoFollower(hardwareMap);
}
```

---

### 2. `GoalPos` API mismatch in Gate autos
**Files:** `BlueCloseGate.java:47`, `RedCloseGate.java:48`, `RedFarGate.java:50`, `BlueFarGate.java:48`

All four active (non-`@Disabled`) Gate autos call the old 2-arg constructor `new GoalPos(x, y)` and the old 4-arg update `goalPos.update(xPos, yPos, bearing, camRange)`. Neither exists — `GoalPos` only has a 3-arg constructor and a 6-arg `update()`.

**Fix:** Update all four files to:
```java
GoalPos goalPos = new GoalPos(x, y, 15.5);
// and in aiming():
goalPos.update(0.08, xPos, yPos, bearing, elevation, camRange);
```

---

### 3. `BlueCloseGate` missing `flyWheel1.setMode(RUN_USING_ENCODER)`
**File:** `BlueCloseGate.java:186`

`setPIDFCoefficients()` is called without first setting the motor mode. The FTC SDK requires `RUN_USING_ENCODER` mode before PIDF coefficients apply — skipping this causes them to be silently ignored and the flywheel runs open-loop.

**Fix:** Add before `setPIDFCoefficients`:
```java
flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
```

---

### 4. `BlueCloseGate` missing `flyWheel2`
**File:** `BlueCloseGate.java`

`RedCloseGate` initializes and drives both `FW1` and `FW2`. `BlueCloseGate` only uses `FW1`. If the hardware has two flywheel motors, the blue auto runs at half power.

---

### 5. `toFWV(double r)` ignores its parameter
**Files:** `RedCloseGate.java:431`, `BlueCloseGate.java:423`, `BlueClose.java:403`, `RedClose.java:401`

The parameter `r` is declared but the method body uses the field `range` directly. This is coincidentally correct when called as `toFWV(range)` but breaks if called with any other argument.

**Fix:**
```java
public double toFWV(double r) {
    return (0.00673 * r * r) + (5.54 * r) + (1162);
}
```

---

### 6. `OfficialRedTeleop.java` and `OfficialBlueTeleop.java` are empty
Both files contain only a blank line. These must be restored or reimplemented.

---

## Part 2 — High Impact Performance

### 7. Flywheel spin-up missing on `OUTTAKE2` in `RedClose15`
**File:** `RedClose15.java`

`idle=true` (which spins up the flywheel during transit) is applied inconsistently:
```
OUTTAKE1 → SHOOT1   move(..., true)  ✓
OUTTAKEB → SHOOTB   move(..., true)  ✓
OUTTAKE2 → SHOOT2   move(...)        ✗  ← flywheel starts from zero
OUTTAKE3 → SHOOT3   move(..., true)  ✓
```
Every `move()` leading to a `SHOOT*` state must pass `idle=true`.

---

### 8. Shoot timeout is time-based, not signal-based
**Files:** All autos — `RedClose15`: 1600ms, Gate autos: 2800ms

The stopper opens when `FWV >= targetV`. The ball exits in under 500ms after that. The remaining 1100–2300ms is wasted waiting. Across 4 shots per auto this costs 4–9 seconds.

**Better exit condition:**
```java
boolean ballGone = (stopper.getPosition() > 0.97) && (flyWheel1.getVelocity() > targetV * 0.98);
if (shotFired && ballGone && actionTimer.getElapsedTime() > 300) {
    // transition early
}
```

---

### 9. Two-step intake paths should be single multi-segment PathChains
**Files:** All autos — `Intake21→Intake22`, `Intake31→Intake32`

Each path boundary costs an `isBusy()` check + 50ms timer + state transition. Merging into one PathChain lets Pedro plan deceleration across both segments:
```java
Intake21 = follower.pathBuilder()
    .addPath(new BezierLine(outtake, intake2p1))
    .addPath(new BezierLine(intake2p1, intake2p2))
    .setConstantHeadingInterpolation(0)
    .setGlobalDeceleration(0.9)
    .build();
```

---

### 10. `setGlobalDeceleration()` unused
**Files:** All autos

Pedro Pathing docs: *"Recommended for use globally, even when path chains are only one path, because it is more optimized than the default mode."* No current auto uses it.

---

### 11. `RedCloseGate.move()` does not hold end position
**File:** `RedCloseGate.java:303`

```java
follower.followPath(path, false);  // no hold — robot drifts
```
`BlueCloseGate` and `RedClose15` use `true`. During `SHOOT*` states the robot must be stationary for accurate aiming. Change to `true` on all outtake paths.

---

### 12. PathConstraints velocity constraint never triggers
**File:** `Constants.java:69`

```java
new PathConstraints(0.99, 100, 1, 1)
// velocity=100 in/s — robot max is ~85 in/s, so this never constrains
// translational=1 inch — loose
```
The FSM relies on `actionTimer > 50ms` rather than true positional accuracy. Tighten:
```java
new PathConstraints(0.99, 0.5, 0.3, 0.01)
```

---

## Part 3 — Robustness

### 13. Servos not initialized in `init()`
**Files:** All autos

`init()` calls `setDirection()` on both servos but never `setPosition()`. If the stopper was previously at 0.973 (open), pieces fall out during the preload drive.

**Fix — add to every auto's `init()`:**
```java
stopper.setPosition(0.9);  // closed
flap.setPosition(0.2);     // default angle
```

---

### 14. `stopper` not closed in `stop()`
**Files:** All autos

If the match ends mid-shot, the stopper stays open (0.973) between rounds.

**Fix — add to every `stop()`:**
```java
stopper.setPosition(0.9);
```

---

### 15. `BlueCloseGate.aiming()` never updates `range`
**File:** `BlueCloseGate.java`

`RedCloseGate.aiming():355` does `range = goalPos.findRange(xPos, yPos)`. `BlueCloseGate.aiming()` skips this. So during path-following, `range` is only current when inside `shoot()`. The turret tracks with a stale range, and the flap angle from `toFWV(range)` uses an outdated value.

---

### 16. First shot fires before vision lock
**Files:** All autos

Until the camera reaches `STREAMING` and `gainSet` becomes true (takes 0.5–3s), the turret targets only from the hardcoded `GoalPos` initial estimate. If the estimate is off and the preload path is short, the first shot fires before any vision correction.

**Mitigation:** Don't leave `PRELOAD` state until `gainSet == true` or a minimum time has elapsed.

---

### 17. Inconsistent turret wrap angle logic
**Files:** `BlueCloseGate.aiming()` vs `RedCloseGate.aiming()`

Blue uses 0–360 wrapping (`> 360+30` / `< 0-30`).
Red uses ±180 wrapping (`> 180+30` / `< -180-30`).

These correspond to different limit conventions (blue: 0→1865, red: -990→850). If wrapping or limits are copied between files without both being updated, the turret will spin the long way around. These should be centralized or clearly documented.

---

### 18. Intake not stopped on time expiry
**Files:** `BlueCloseGate.java`, `RedCloseGate.java`

The 29.3/29.5s cutoff block stops flywheels and returns the turret, but does not call `intake.setPower(0)`. If the match ends while in a `moveIntake` state, the intake motor runs past the match end.

---

## Part 4 — Vision System

### 19. Debug rendering overlays enabled in competition
**Files:** All autos — `initWebcam()`

```java
.setDrawAxes(true)
.setDrawCubeProjection(true)
.setDrawTagOutline(true)
```
These render 3D overlays onto every camera frame. With `enableLiveView(false)`, the output is never seen. Set all three to `false` to reduce CPU load.

---

### 20. Vision alpha never snapshots on first detection
**Files:** Gate autos

`goalPos.update(0.08, ...)` with alpha=0.08 takes ~35 frames (1.2 seconds) to converge 95% of the way. `FlywheelTesting.java` already demonstrates the correct pattern — use alpha=1.0 on the first detection to snap immediately, then fall back to 0.08:
```java
double alpha = hasEst ? 0.08 : 1.0;
goalPos.update(alpha, xPos, yPos, bearing, elevation, camRange);
hasEst = true;
```

---

### 21. Camera offset constants are undocumented magic numbers
**Files:** All autos

`camOffsetX = 2` is applied to range. Bearing corrections use `Math.atan(3.2/range)` in `RedClose15` and `Math.atan(2.5/range)` in Gate autos with no documentation on what physical measurement each value represents. If the camera is remounted, it is not clear which numbers need to change.

---

## Part 5 — Architecture & Maintainability

### 22. No shared base class for autonomous OpModes
All 9 autos duplicate `initWebcam()`, `cameraControls()`, `move()`, `moveIntake()`, `shoot()`, `aiming()`, and `toFWV()`. A bug fixed in one file must be manually fixed in all others — this is how `toFWV` ended up with a parameter bug in 4 files while being correct in 1.

A `BaseAuto` abstract class with abstract methods for side-specific values (tag ID, goal position, bearing sign, turret limits) would eliminate the duplication.

---

### 23. `DecodeDriveTrain` subsystem unused in all autos
The subsystem was created for code reuse but every auto initializes and drives motors directly. It is only used in `FlywheelTesting.java`. Motor configuration (names, directions) is duplicated between `DecodeDriveTrain.java` and `Constants.java`.

---

### 24. `DecodeDriveTrain.pose2D` is never initialized
**File:** `DecodeDriveTrain.java:163`

`pose2D` is declared but never assigned. If any caller passes `showTelemetry = true`, the method throws a NullPointerException on `pose2D.getX()`.

---

### 25. PID gains undocumented across files

| File | P value |
|------|---------|
| `RedClose15` | 400 |
| Gate autos (Red) | 400 |
| Gate autos (Blue) | 380 |
| `FlywheelTesting` | 200 |

No documentation explains why these differ. The test mode value (200) is drastically lower than competition values, which is appropriate, but the 380 vs 400 difference between blue and red is unexplained.

---

### 26. Legacy files polluting the codebase
The following files are no longer used and should be removed or kept `@Disabled` and moved to an archive folder:
- `BlueSideClose.java`, `BlueSideFar.java`, `RedSideClose.java`, `RedSideFar.java`
- `BlueTeleopWebcam.java`, `RedTeleopWebcam.java`
- `centerstageTeleOp.java`
- `TestAuto.java`

`BlueTeleopWebcam.java` and `RedTeleopWebcam.java` also call `Constants.createFollower()` (the missing method) which contributes to the build failure.

---

## Priority Order

| Priority | Items | Effort |
|----------|-------|--------|
| **P0 — Fix to build** | 1, 2, 3, 4, 5, 6 | Low |
| **P1 — Match performance** | 7, 8, 9, 10, 11 | Medium |
| **P2 — Reliability** | 12, 13, 14, 15, 16, 17, 18 | Low–Medium |
| **P3 — Vision accuracy** | 19, 20, 21 | Low |
| **P4 — Architecture** | 22, 23, 24, 25, 26 | High |
