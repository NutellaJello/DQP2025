# Maximum-Performance FTC Robot Plan
**Date:** 2026-05-02

This document is the strategic roadmap for making this robot the fastest, most accurate, and most reliable robot the rules allow. It assumes the build-breaking bugs in `CODE_REVIEW.md` are already fixed and the library upgrades in `LIBRARY_REVIEW.md` are already considered.

The plan is organized in **tiers** — Tier 0 must be done first, Tier 1 gives the biggest wins per hour of work, and later tiers are polish. At competition you compete with the entire stack. At each tier, "Why" tells you what physical effect on the robot we're chasing.

---

## Tier 0 — Fix the Build (Prerequisite)

You cannot tune what doesn't compile. `CODE_REVIEW.md` lists 6 P0 build-breaking bugs (missing `createFollower`, wrong `GoalPos` constructor calls, missing `setMode` before `setPIDFCoefficients`, missing `flyWheel2` in `BlueCloseGate`, empty `OfficialRedTeleop`/`OfficialBlueTeleop`). Fix every P0 before continuing.

---

## Tier 1 — The "10× Wins" (Do These First)

These changes give the biggest performance gain per hour of effort. Doing all four turns this robot from a typical FTC bot into a top-tier one.

### 1.1 LynxModule Bulk Read — **biggest single code change you can make**

**What is it?** The REV Control Hub talks to each motor and encoder over an internal high-speed bus. Normally, every time your code calls something like `motor.getCurrentPosition()` or `motor.getVelocity()`, the SDK sends one bus request, waits for the answer, and returns. Each round-trip costs roughly 1 millisecond. A typical autonomous loop reads 4 drive encoders, the Pinpoint, the turret encoder, both flywheel velocities, etc. — that's 8–10 round trips per loop, or about 8–10 ms of pure waiting time per loop.

**Bulk caching mode** tells the SDK: "Don't send a request every time I ask. Instead, every loop, send ONE big request that grabs all motor data in a single transaction, and serve every subsequent read from that cache." One round trip per loop instead of ten.

**Why it matters:** Loop frequency typically jumps from **~80 Hz to ~240 Hz** with this single change. That means:
- Pedro Pathing's PID corrections happen 3× more often → tighter path following, less overshoot, higher safe top speed.
- The turret's targeting updates 3× more often → less aiming lag while the robot is moving.
- The flywheel velocity controller reacts 3× faster → faster recovery after a shot launches.
- The shoot-ready check (`isFW1Ready && isFW2Ready`) becomes 3× more responsive.

**How to add it:** In `init()` of every OpMode (or in a shared base class), add:

```java
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

private List<LynxModule> hubs;

// In init():
hubs = hardwareMap.getAll(LynxModule.class);
for (LynxModule hub : hubs) {
    hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
}

// In every loop() iteration, BEFORE reading any motor data:
for (LynxModule hub : hubs) {
    hub.clearBulkCache();   // forces ONE bulk read on the next motor get*() call
}
```

**`MANUAL` vs `AUTO`:** `AUTO` clears the cache automatically the first time you call a `get*()` method per loop, which is convenient but means if you accidentally call `getCurrentPosition()` twice in the same loop, the second call triggers a fresh bus read. `MANUAL` is strictly faster — you call `clearBulkCache()` exactly once per loop and every subsequent read in that loop is free.

**Estimated impact:** +160 Hz loop rate, ~10 ms saved per loop. This unlocks every other optimization in this document.

---

### 1.2 Limelight 3A Vision Coprocessor

Already covered in detail in `LIBRARY_REVIEW.md` Recommendation 1. Recap of why it's Tier 1:
- Removes all vision computation from the Control Hub CPU — no more competition between vision and Pedro Pathing for processor time.
- Eliminates the 1–3 second `gainSet` warm-up that currently means the first shot may fire blind.
- Produces less noisy pose estimates, which lets you replace the slow EMA filter in `GoalPos` (currently takes 1.2 s to converge) with direct measurements or a faster filter.
- Web dashboard for real-time exposure tuning at competition — no more rebuilding the APK to fix lighting.

**Estimated impact:** First-shot accuracy goes from "depends on luck" to "consistently on target." Turret tracking lag during fast motion drops from ~200 ms to under 50 ms.

---

### 1.3 Pedro Pathing Predictive Braking

Already covered in `LIBRARY_REVIEW.md` Recommendation 2. Recap of why it's Tier 1:
- ~15% faster autonomous paths (per Pedro's docs, world-record team result).
- No more hand-tuning translational/drive PIDFs — the auto-tuner outputs `kLinear` and `kQuadratic` for you.
- Lower `tValueConstraint` (drop from 0.99 to 0.95) lets actions trigger sooner because predictive braking actually stops on time instead of overshooting.
- Combined with `centripetalScaling(0)` because predictive braking already accounts for centripetal forces.

**Estimated impact:** A 28-second autonomous becomes a 24-second autonomous, leaving 4 extra seconds for additional cycles.

---

### 1.4 Pedro Pathing Ivy — Parallel Mechanism Execution

Already in `LIBRARY_REVIEW.md` Recommendation 5. Why it's Tier 1: every shoot cycle currently runs **sequentially** — drive to outtake position → arrive → spin flywheels → wait for them to reach target velocity → shoot. The flywheel spin-up takes about 800 ms.

With Ivy, the flywheel spin-up runs **in parallel** with the drive segment:

```java
parallel(
    follow(follower, OuttakePath),
    run(() -> {
        flyWheel1.setVelocity(targetV);
        flyWheel2.setVelocity(targetV);
    })
)
```

By the time the robot arrives at the outtake position, the flywheels are already at speed. **0.8 s saved per shoot cycle × 4 cycles per auto = ~3 seconds reclaimed.**

---

## Tier 2 — Hardware & Wiring Architecture

These are physical-robot decisions. Doing them wrong caps the performance of every software optimization above.

### 2.1 USB Port Layout

**Rule:** Webcam (or Limelight) goes on the Control Hub's **USB 3.0 port**, never USB 2.0. The USB 2.0 port shares its bus with the Control Hub's WiFi radio. Heavy USB traffic on that port causes WiFi disconnects and ESD-induced webcam dropouts mid-match. The USB 3.0 port has dedicated bandwidth.

**Why this matters:** A webcam that disconnects for half a second mid-autonomous loses lock on the AprilTag, and `gainSet` may have to re-trigger on the next OpMode. You lose accuracy and time on a single bad cable plug.

### 2.2 I²C Bus Separation

**Rule:** The Pinpoint odometry computer must NOT share an I²C bus with another device that has firmware glitches. **Avoid I²C Port 0** — it shares the bus with the onboard IMU. Put the Pinpoint on Port 1, 2, or 3.

Why this matters: when two devices share a bus and one of them does a bad read (`FAULT_BAD_READ` in the Pinpoint status), it can stall the entire bus for the other. Pedro Pathing depends on Pinpoint being polled at 1 kHz internally — bus stalls translate directly into localization drift.

### 2.3 Powered USB Hub for Limelight

If you add the Limelight 3A, plug it into a **powered external USB hub** (e.g., Acer ODK350) connected to the Control Hub USB 3.0 port, with the hub powered from an external source. The Limelight draws constant power because its onboard processor is always running. Powering it directly from the Control Hub draws from the same battery rail as the motors and can cause brownouts during high-current motor demands (e.g., flywheel spin-up + drive acceleration simultaneously).

### 2.4 Battery Discipline

NiMH batteries lose ~1% charge per day to self-discharge and lose internal capacity over their lifetime. For consistent autonomous performance:
- Charge the day before competition, not the morning of.
- Track cycle counts per battery; retire batteries above ~300 cycles.
- Measure internal resistance every 200–300 cycles. A battery with high internal resistance reads "full" on a multimeter but sags hard under flywheel load, causing inconsistent shot velocity.
- Use the same battery for tuning and for the actual match. Tuning your shooter with a fresh battery and running matches with a tired battery means your shots fall short.

### 2.5 Servo Power Module (Optional)

If your shooter uses high-torque servos under heavy load (the flap, the stopper) that occasionally cause the entire robot to brown out, install a servo power module with proper isolation. Without isolation, a stalled servo can pull the bus voltage low enough to reset the Control Hub.

---

## Tier 3 — Code Hot-Path Optimization

After Tier 1's bulk read, your loop is fast. These changes keep it fast by removing wasted work inside the loop. Each one is small individually; together they save several milliseconds per loop, which compounds with bulk read to push loop hertz to 300+.

### 3.1 Cache Hardware Reads Per Loop

Even with bulk caching, calling `turret.getCurrentPosition()` three times per loop returns the same value three times — but each call goes through SDK plumbing. Read once at the top of the loop, reuse:

```java
// At top of loop():
int turretPos = turret.getCurrentPosition();
int fw1Velocity = (int) flyWheel1.getVelocity();
int fw2Velocity = (int) flyWheel2.getVelocity();

// Use the cached locals everywhere downstream — never call get*() again this loop.
```

Currently `RedClose15.java:321,396` reads turret position twice per loop in different methods.

### 3.2 Skip Redundant Servo & Motor Writes

Every `servo.setPosition()` call sends a command over the bus, even if the position you're sending is the same as the last one. The servo doesn't move, but the bus traffic still happens. For the flap (which only changes when range crosses thresholds) and stopper (which only changes between shots), this is dozens of wasted writes per second.

Pattern to apply:

```java
private double lastFlapPos = -1;
private int lastTurretTarget = Integer.MIN_VALUE;

// Wherever you currently call flap.setPosition(flapPos):
if (flapPos != lastFlapPos) {
    flap.setPosition(flapPos);
    lastFlapPos = flapPos;
}

// Wherever you currently call turret.setTargetPosition(turretTarget):
if (turretTarget != lastTurretTarget) {
    turret.setTargetPosition(turretTarget);
    lastTurretTarget = turretTarget;
}
```

Apply to: `flap`, `stopper`, `intake` (servo), `turret.setTargetPosition`. Found redundant writes at `RedClose15.java:367–374, 420`, `BlueFar.java:298–325`, and elsewhere.

### 3.3 Disable Telemetry in Autonomous

Several auto OpModes call `telemetry.update()` every loop with all `telemetry.addData()` lines commented out (see `RedClose15.java:240–245`, `BlueCloseGate.java:226`). `telemetry.update()` still serializes and sends the empty packet over the network — it's not free. Either:
- Comment the `telemetry.update()` calls too if you don't need them, or
- Wrap them in `if (DEBUG)` guards with a static final boolean.

In autonomous, you don't read telemetry — the match is happening. Save the bandwidth.

### 3.4 Disable Vision Processor When Not Aiming

The AprilTag processor consumes CPU on every frame even if your code isn't reading detections. During the drive segments where the robot is moving from intake back to outtake and the camera isn't pointed at the goal, the processor is wasting cycles.

```java
visionPortal.setProcessorEnabled(aprilTagProcessor, true);   // when entering aim phase
visionPortal.setProcessorEnabled(aprilTagProcessor, false);  // during drive segments
```

This is **instant** — unlike `stopStreaming()`, which has a ~1 second delay. With Limelight (Tier 1), this is moot, but until then, it helps.

### 3.5 Pre-compute Trig Where Possible

`Math.atan(3.2/range)` and `Math.toRadians(...)` are called on every aiming loop. Where the inputs change slowly:
- Cache the last `range`, only recompute `Math.atan(3.2/range)` when range changes by > 0.5 inches.
- Pre-compute Math.toRadians-style conversions as final constants where the input is a literal: `private static final double DEG_TO_RAD = Math.PI / 180.0;`

These are tiny on their own but compound when called every frame.

### 3.6 Cache Timer Reads

`actionTimer.getElapsedTime()` is called 3+ times per loop in `RedClose15.java` state branches. Each call hits `System.nanoTime()`. Read once at the top:

```java
double elapsed = actionTimer.getElapsedTime();
// Use `elapsed` everywhere downstream instead of calling getElapsedTime() repeatedly.
```

---

## Tier 4 — Pedro Pathing Advanced Tuning

After Predictive Braking (Tier 1.3) is in place, these features squeeze more performance out of Pedro.

### 4.1 PathChains over Individual Paths

Pedro Pathing has two construction patterns:
- `Path` — a single segment. The robot fully stops at the end before the next path starts.
- `PathChain` — multiple connected segments. The robot maintains momentum through transition points.

Predictive Braking comes to a complete stop at every individual `Path` end. If your auto is built as five separate `follower.followPath(path)` calls, the robot stops 5 times. Built as one `PathChain`, the robot maintains speed through the curves.

**Where to apply:** Anywhere two consecutive paths don't require a full stop in between (drive → drive, drive → curve back). The `Outtake → Intake → Outtake` loop is a perfect candidate. Reformulate as `PathChainBuilder` with each segment as `.addPath(...)`.

**Estimated impact:** Another 1–2 seconds off autonomous on top of Predictive Braking.

### 4.2 Path Callbacks for Mid-Path Actions

Currently, mechanism actions (lower the intake, spin up the flywheel) happen between path calls. With path callbacks, you can fire an action at a specific percentage of a path:

```java
follower.followPath(
    pathChain
        .addParametricCallback(0.6, () -> flyWheel1.setVelocity(targetV))  // start spin-up at 60% of path
        .addParametricCallback(0.9, () -> intake.setPosition(INTAKE_DOWN))  // drop intake at 90%
        .build()
);
```

This is a more flexible alternative to Ivy for simple "do X at point Y" needs. Use callbacks for one-off triggers, Ivy for full parallel command graphs.

### 4.3 Global Deceleration with BrakingStart

Pedro 2.0.5's global deceleration extends the braking calculation across an entire `PathChain` instead of per-path. `BrakingStart` controls when deceleration begins as a fraction of remaining distance. Tuning `BrakingStart` lower (start braking earlier) gives smoother stops at the cost of slightly slower paths; higher gives more aggressive late braking.

**Recommendation:** Always set `setGlobalDeceleration()` on every `PathChain`, even single-segment ones. It's strictly better than the default per-path deceleration.

### 4.4 Filtered PID + Kalman Tuning

Pedro's `FilteredPIDFCoefficients(P, I, D, T, F)` includes a time constant `T` (default 0.6) that smooths the derivative term — `D_effective = 0.6 × previous + 0.4 × current`. For a robot with noisy localization (encoder pods on uneven carpet), increase `T` toward 0.7–0.8 for smoother control. For a robot with very clean localization (Pinpoint on flat tile), drop `T` to 0.4–0.5 for snappier corrections.

Pedro also exposes Kalman filter parameters (`modelCovariance`, `dataCovariance`) on the drive PID. Higher `dataCovariance` makes the filter trust raw measurements more (faster response to real changes); higher `modelCovariance` makes it trust the model more (smoother but laggier).

### 4.5 Per-Path Maximum Power

Most paths can run at full power; tight curves near walls cannot without overshoot. Use `follower.followPath(chain, maxPower, holdEnd)` to throttle individual sections:

```java
follower.followPath(wallApproach, 0.6, true);   // slow on the wall side
follower.followPath(openField, 1.0, true);      // full speed across the middle
```

Better than tuning a global `maxPower` for the worst case, which leaves performance on the table everywhere else.

### 4.6 Drop tValueConstraint to 0.95

After Predictive Braking is tuned, lower `tValueConstraint` from `0.99` to `0.95` in `PathConstraints`. Why: under PIDF, the robot tends to overshoot and hit `t=1.0` early, so the high constraint catches it; under Predictive Braking, the robot stops cleanly, so the constraint must be lowered to avoid waiting at the end of every path.

### 4.7 Verify the velocity Constraint

`PathConstraints(0.99, 100, 1, 1)` has `velocity = 100` in/s — but this robot's max velocity is around 85 in/s, so the velocity criterion **never fires**. Drop it to 1–2 in/s so it actually contributes to "path complete" detection. (This is also documented in `CODE_REVIEW.md`.)

---

## Tier 5 — Algorithm Improvements

These are upgrades to the math, not the framework. They make the robot smarter, not just faster.

### 5.1 Lead the Target

Currently `aiming()` points the turret at where the goal **is**. By the time the projectile reaches the goal (~0.4 s flight time), the robot has moved. While the robot is stationary at the shoot position this doesn't matter, but during driving shots (if you ever take them) it does.

A simple lead correction:

```java
// Robot velocity from Pinpoint (in/s):
double vx = pinpoint.getVelX();
double vy = pinpoint.getVelY();

// Predict goal position at projectile arrival time:
double flightTime = camRange / projectileSpeed;
double leadX = goalPos.x - vx * flightTime;
double leadY = goalPos.y - vy * flightTime;

// Aim at leadX/leadY instead of goalPos.x/y.
```

This is a few lines of code and unlocks shooting on the move.

### 5.2 Replace EMA with Kalman Filter

Currently `GoalPos.update()` uses a fixed alpha EMA: `a = a*(1-alpha) + a2*alpha`. EMA's tradeoff is fixed — low alpha means smooth but laggy; high alpha means responsive but noisy.

A Kalman filter adapts the smoothing dynamically: when measurements are consistent, it trusts them more; when they're jumpy, it trusts the model more. For a goal that's stationary but produces noisy camera readings, a 1-D Kalman per axis (x, y, z) is ~30 lines of code and gives you both fast convergence AND noise rejection.

This becomes especially valuable with Limelight, which produces less noisy data — the Kalman can trust raw readings much more.

### 5.3 Range-Adaptive Vision

The AprilTag processor's decimation parameter trades resolution for speed. At long range (where you need precise pose), use decimation = 1 or 2. At short range (where the tag is huge in frame), decimation = 3 or 4 is plenty and runs ~2× faster.

Switch dynamically:

```java
if (camRange > 100) aprilTag.setDecimation(2);
else aprilTag.setDecimation(3);
```

Saves CPU during the close-range cycles where vision precision is over-spec.

### 5.4 Voltage-Compensated Flywheel Feedforward

Flywheel velocity changes with battery voltage. A flywheel that holds 4500 rpm at 13.5 V will only hold ~4200 rpm at 12 V with the same `F` value. Compensate:

```java
double battery = hardwareMap.voltageSensor.iterator().next().getVoltage();
double f = F_NOMINAL * (13.5 / battery);
flyWheel1.setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, f));
```

This keeps shot consistency across the match as battery sags, and across matches as different batteries are used.

### 5.5 Adaptive Flap Mapping

Currently flap position is set by hardcoded range thresholds (`FLAP_NEAR`, `FLAP_MID`, `FLAP_FAR`). For more accuracy, fit a continuous function. After collecting 5–10 (range, optimal-flap) pairs in testing, fit a polynomial:

```java
// Cubic fit example — coefficients from polyfit in Excel/Python:
double flapPos = a*range*range*range + b*range*range + c*range + d;
flap.setPosition(clamp(flapPos, FLAP_MIN, FLAP_MAX));
```

Smooth flap interpolation lets you shoot from any range, not just the three pre-tuned spots.

---

## Tier 6 — Driver Experience (TeleOp)

Currently `OfficialRedTeleop.java` and `OfficialBlueTeleop.java` are empty files — TeleOp is the missing half of the season. When you build them, here are the high-leverage features to include from day one:

### 6.1 Driver-Assist Auto-Aim

Hold a button (e.g., right bumper) to enable automatic turret tracking. The driver focuses on positioning the chassis; the turret + flap + flywheel speed are computed automatically based on Limelight pose. This is the same `aiming()` logic from autonomous, just gated by a button hold.

### 6.2 Quick-Action Macros

Map a button to "drive to nearest scoring position and shoot 3 balls" — Pedro Pathing can run a short PathChain in TeleOp the same way it does in auto. Useful for endgame rushes where every second matters.

### 6.3 Adjustable Drive Curves

Apply a cubic curve to gamepad sticks so small inputs are gentle and full deflection is fast: `output = sign(x) * x²`. Better than linear because most of the driving happens at low input where precision matters.

### 6.4 Field-Centric Drive

If the chassis is mecanum, use the Pinpoint heading to make "forward on the stick = away from the driver" regardless of robot rotation. Drivers' reaction time improves dramatically because they don't have to mentally rotate the robot's frame.

### 6.5 FTC Dashboard Telemetry

Already in `LIBRARY_REVIEW.md` Recommendation 4. For TeleOp tuning of flywheel and flap mapping, this is the difference between an iteration cycle of 30 seconds and 3 minutes.

---

## Recommended Implementation Order

The order matters because each tier unlocks the next. Doing Tier 4 path tuning before Tier 1 bulk read means you'll re-tune everything once loop hertz changes.

| Step | Task | Effort | Cumulative Impact |
|------|------|--------|------|
| 1 | Fix all P0 build bugs (`CODE_REVIEW.md`) | 4 hours | Baseline |
| 2 | Add `LynxModule.setBulkCachingMode(MANUAL)` to all OpModes | 30 minutes | +3× loop hertz |
| 3 | Add hardware-write deduplication helpers (Tier 3.2) | 1 hour | +20–30 Hz loop |
| 4 | Tune Predictive Braking (`LIBRARY_REVIEW.md` 2) | 2 hours | -15% auto time |
| 5 | Drop `tValueConstraint` to 0.95, fix `velocity=100` bug | 10 minutes | -1 s per path end |
| 6 | Add FTC Dashboard (`LIBRARY_REVIEW.md` 4) | 1 hour | Tuning velocity 5× faster |
| 7 | Add `PathChain` reformulation for outtake/intake loop | 2 hours | -1 s per cycle |
| 8 | Implement Ivy parallel flywheel spin-up | 2 hours | -0.8 s per shot |
| 9 | Add path callbacks for intake drop / flywheel start | 1 hour | -0.5 s per cycle |
| 10 | Wire Limelight 3A hardware + integrate (`LIBRARY_REVIEW.md` 1) | 1–2 days | First-shot accuracy, no warm-up |
| 11 | Replace EMA with Kalman in `GoalPos` | 4 hours | Faster lock, less jitter |
| 12 | Voltage-compensated flywheel feedforward | 1 hour | Cross-match shot consistency |
| 13 | Lead-the-target turret correction | 2 hours | Shoot-on-the-move capability |
| 14 | Implement TeleOp driver-assist + macros | 2 days | Match performance ceiling |

**Estimated total autonomous time savings:** A current ~28-second auto becomes a ~20-second auto, which means time for an extra 1–2 cycles per match.

**Estimated TeleOp impact:** Going from "no driver assist" to "auto-aim + macros" typically lifts cycle time from 6 seconds per cycle to 4 seconds per cycle — that's the difference between 25 cycles and 38 cycles in a 2:30 driver-controlled period.

---

## What We're NOT Recommending and Why

- **Switching from mecanum to swerve.** Swerve adds 8–12 modules of complexity, requires custom firmware, and is not currently compatible with Pedro Pathing's Predictive Braking. The performance gap is real but the build cost is enormous. Not worth it for a single season unless you have a dedicated module-builder.
- **Custom AprilTag detection (e.g., OpenCV pipeline).** Limelight or the SDK's built-in detector is faster, more accurate, and field-tested. Rolling your own is a multi-month project that almost certainly performs worse.
- **Switching off Pinpoint to OTOS.** Pinpoint with proper offsets and reset discipline is essentially drift-free on FTC tile. OTOS is also good but is a lateral move, not an upgrade. Only consider if the team observes localization drift mid-match (per `LIBRARY_REVIEW.md` 6).
- **Threading vision off the main loop manually.** Adding multithreading inside FTC OpModes is risky (lifecycle, hardware-map ownership). Limelight gives you the same benefit without writing any thread code.
- **Building a separate coprocessor PC.** Limelight already is one, with FTC-legal hardware and an integration library. A custom Pi or Jetson is not cost-effective.
