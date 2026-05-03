# Logging Recommendations
**Date:** 2026-05-02

Good logging is the difference between "something went wrong in that run" and "the camera didn't lock until 2.8 seconds, so the first shot fired blind, and the turret was 11 ticks off at the moment of release." This document covers what to log, how to log it cheaply, and where the current codebase is flying blind.

---

## The Core Problem: Everything Fails Silently

Right now there are at least six things that can go wrong in a match with **zero visible indication**:

| Failure | Current visibility | Why it's invisible |
|---|---|---|
| Camera warms up after first shot | None | `gainSet` flag set silently |
| Turret hits limit and stops moving | None | Limits clamp silently |
| Flywheel never reaches target velocity | None | Timeout fires regardless |
| GoalPos EMA hasn't converged | None | `aiming()` runs regardless |
| Path t-value triggers before robot is positioned | None | No path end logging |
| Loop hertz drops below 100 Hz | None | Nothing tracks this |

Logging fixes this by producing a timeline you can replay after a match.

---

## Three Logging Modes

Use all three simultaneously — they serve different purposes.

### Mode 1 — Driver Station Telemetry (Competition-Safe)

`telemetry.addData()` lines are essentially free — they just write to a string buffer. `telemetry.update()` is what costs time (serializes the buffer, sends a WiFi packet). 

**Pattern: always-on state logging, update rate limited.**

```java
// At the top of every OpMode, add:
private static final boolean VERBOSE = false;     // flip to true in practice
private long lastTelemetryUpdate = 0;

// In loop(), replace bare telemetry.update() with:
if (VERBOSE || System.currentTimeMillis() - lastTelemetryUpdate > 200) {
    telemetry.update();
    lastTelemetryUpdate = System.currentTimeMillis();
}
```

This sends telemetry at most 5× per second in competition, instead of 200+ times per second. The driver station refreshes at ~4 Hz anyway — you lose nothing and save ~4 ms/loop.

**What to always include (even `VERBOSE=false`):**

```java
telemetry.addData("State", currentPathState.toString());
telemetry.addData("Loop Hz", String.format("%.0f", loopHz));   // see Mode 3 below
telemetry.addData("FW1 vel", String.format("%.0f", fw1Velocity));
telemetry.addData("FW2 vel", String.format("%.0f", fw2Velocity));
telemetry.addData("Turret", String.format("%d → %d", turretPos, turretTarget));
```

Five lines. These alone let you see if the robot is stuck in the wrong state, if the flywheels are at speed, and if the turret is converging or stalled at a limit.

**What to add only when `VERBOSE=true` (practice only):**

```java
if (VERBOSE) {
    telemetry.addData("gainSet", gainSet);
    telemetry.addData("camRange", String.format("%.1f in", camRange));
    telemetry.addData("GoalPos", String.format("(%.1f, %.1f)", goalPos.a, goalPos.b));
    telemetry.addData("xPos yPos", String.format("(%.1f, %.1f)", xPos, yPos));
    telemetry.addData("bearing deg", String.format("%.1f", Math.toDegrees(bearing)));
    telemetry.addData("tValue", String.format("%.3f", follower.getCurrentTValue()));
    telemetry.addData("Battery V", String.format("%.2f V", batteryVoltage));
    telemetry.addData("detections", detectedTags != null ? detectedTags.size() : 0);
}
```

---

### Mode 2 — FTC Dashboard Graphs (Practice Tuning)

FTC Dashboard (`com.acmerobotics.dashboard`, already recommended in `LIBRARY_REVIEW.md`) has a **Telemetry** panel with live graphs. Unlike Driver Station telemetry, it transmits over a separate UDP channel at high frequency without impacting loop timing.

**Most useful graphs to add:**

```java
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// At top of class:
FtcDashboard dashboard = FtcDashboard.getInstance();
TelemetryPacket packet = new TelemetryPacket();

// In loop(), before telemetry.update():
packet.put("FW1 velocity", flyWheel1.getVelocity());
packet.put("FW2 velocity", flyWheel2.getVelocity());
packet.put("FW1 target", toFWV(range));
packet.put("Turret pos", turretPos);
packet.put("Turret target", turretTarget);
packet.put("Turret error", turretTarget - turretPos);
packet.put("Loop Hz", loopHz);
packet.put("GoalPos X", goalPos.a);
packet.put("GoalPos Y", goalPos.b);
dashboard.sendTelemetryPacket(packet);
```

**What the graphs reveal:**

- **Flywheel velocity vs target graph:** See if both flywheels are tracking together, or if FW2 (the one missing in `BlueCloseGate`) is missing. See if they sag after a shot. See how long spin-up actually takes vs how long you're waiting.
- **Turret error graph:** See if the turret oscillates (P too high), creeps slowly (P too low), or hits limits (error jumps to a constant).
- **Loop Hz graph:** See exactly when the loop slows down — during Pedro Pathing curves, during AprilTag processing, etc. This tells you where to focus optimization.

---

### Mode 3 — Event Log File (Post-Match Analysis)

For competition, you can't watch a screen during the match. An event log writes key moments to a file on the Control Hub's storage — you read it on a laptop in the pit afterwards.

**Implementation:**

```java
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

private BufferedWriter matchLog;
private long matchStartTime;

// In init():
try {
    matchLog = new BufferedWriter(new FileWriter(
        "/sdcard/FIRST/matchlog_" + System.currentTimeMillis() + ".txt"
    ));
    matchStartTime = System.currentTimeMillis();
    log("Init complete");
} catch (IOException e) { /* silently skip if storage unavailable */ }

// Helper method:
private void log(String message) {
    if (matchLog == null) return;
    try {
        long elapsed = System.currentTimeMillis() - matchStartTime;
        matchLog.write(String.format("[%6.2f s] %s\n", elapsed / 1000.0, message));
        matchLog.flush();
    } catch (IOException ignored) {}
}

// In stop():
try { if (matchLog != null) matchLog.close(); } catch (IOException ignored) {}
```

**What to log (state transitions + key events only, NOT per-loop):**

```java
// Log state transitions in move():
private void move(PathChain path, PathState nextState) {
    follower.followPath(path, true);
    log("→ " + nextState.name() + " | pos ("+String.format("%.1f",xPos)+", "+String.format("%.1f",yPos)+")");
    pathState = nextState;
}

// Log when gainSet becomes true:
gainSet = true;
log("Camera locked | elapsed " + opmodeTimer.getElapsedTime() + "ms");

// Log when aiming() gets a detection:
log("Tag detected | range " + String.format("%.1f", camRange) + " | bearing " + String.format("%.1f", Math.toDegrees(bearing)));

// Log at shot moment:
log("SHOOT | FW1=" + (int)flyWheel1.getVelocity() + " FW2=" + (int)flyWheel2.getVelocity()
    + " | turretErr=" + (turretTarget-turretPos)
    + " | goalPos ("+String.format("%.1f",goalPos.a)+", "+String.format("%.1f",goalPos.b)+")");

// Log path completion:
log("Path complete | tValue=" + String.format("%.3f", follower.getCurrentTValue()));

// Log timeout hits (vs normal completion):
if (actionTimer.getElapsedTime() > SHOOT_TIMEOUT) {
    log("TIMEOUT in " + pathState.name() + " after " + actionTimer.getElapsedTime() + "ms");
}
```

**Reading the log in the pit:**

Connect to the Control Hub via ADB:
```bash
adb pull /sdcard/FIRST/matchlog_<timestamp>.txt .
```
Or use the RC app's log viewer. A typical log looks like:

```
[  0.00 s] Init complete
[  0.12 s] → SHOOTPRE | pos (0.0, 0.0)
[  1.84 s] Camera locked | elapsed 1842ms
[  2.31 s] Tag detected | range 48.2 | bearing -3.1
[  2.44 s] → SHOOT1 | pos (24.3, 87.1)
[  2.52 s] SHOOT | FW1=4432 FW2=4418 | turretErr=3 | goalPos (147.2, 143.8)
[  4.17 s] TIMEOUT in SHOOT1 after 1600ms        ← first shot timed out
[  4.19 s] → INTAKE1 | pos (24.1, 87.0)
```

This one log tells you: camera took 1.8 s to lock, first shot went at FW2=4418 (target was probably 4500 — slightly low), turret was only 3 ticks off (not the problem), shot timed out at 1600 ms. You know to investigate the flywheel not reaching target velocity, not the turret.

---

## Loop Hertz Measurement

Add this to every OpMode — it's the single most useful performance diagnostic:

```java
// Fields:
private double loopHz = 0;
private long lastLoopTime = 0;

// At the TOP of every loop() iteration (before anything else):
long now = System.nanoTime();
if (lastLoopTime != 0) {
    loopHz = 1_000_000_000.0 / (now - lastLoopTime);
}
lastLoopTime = now;

// Then after adding bulk caching (from PERFORMANCE_PLAN.md Tier 1.1):
for (LynxModule hub : hubs) hub.clearBulkCache();
```

Target loop hertz before optimizations: **~80 Hz**.
After `setBulkCachingMode(MANUAL)`: **~200–250 Hz**.
After hardware-write deduplication: **~280–320 Hz**.

If you see loop hertz drop to 50 Hz during a specific state, that state has a hardware read or heavy computation causing a bottleneck. The log will show exactly when.

---

## Specific Blind Spots to Fix in the Current Code

These are places in the existing autonomous files where something can fail with no indication:

### 1. `gainSet` timing — how long did the camera take?

`RedClose15.java` and all other autos silently set `gainSet=true` with no log of how long it took. Add:

```java
// In cameraControls(), where gainSet = true:
gainSet = true;
log("Camera STREAMING after " + opmodeTimer.getElapsedTime() + "ms");
// Also add to telemetry:
telemetry.addData("gainSet at", opmodeTimer.getElapsedTime() + "ms");
```

If this number is ever > 2000 ms, the first shot fires before the camera has valid data. You'll only know this with a log.

### 2. AprilTag detection gaps — how many frames had no tag?

Currently `aiming()` runs the EMA only when a tag is detected. If the camera loses sight of the tag for 500 ms, the turret aims at a stale position with no indication. Add:

```java
private int consecutiveMissFrames = 0;
// In aiming(), in the `for (AprilTagDetection detection : ...)` block:
if (detectedTags.isEmpty()) {
    consecutiveMissFrames++;
    if (consecutiveMissFrames > 10) {
        log("Tag lost for " + consecutiveMissFrames + " frames");
    }
} else {
    consecutiveMissFrames = 0;
}
```

### 3. Turret limit hits — why is the turret not moving?

If `turretTarget` is outside `[lowLimit, highLimit]`, the turret silently stops. Add:

```java
// Wherever turretTarget is clamped to limits:
if (turretTarget < lowLimit || turretTarget > highLimit) {
    log("Turret clamped: target=" + turretTarget + " limits=[" + lowLimit + "," + highLimit + "]");
    turretTarget = Math.max(lowLimit, Math.min(highLimit, turretTarget));
}
```

### 4. Flywheel velocity at shot — was the flywheel ready?

The `shoot()` method fires after a timeout or when velocity is in range, but doesn't log which exit condition was hit or what the actual velocity was:

```java
// At the moment the stopper fires (the actual shot):
log("Shot fired | FW1=" + (int)flyWheel1.getVelocity()
    + " FW2=" + (int)flyWheel2.getVelocity()
    + " | target=" + (int)toFWV(range)
    + " | elapsed=" + actionTimer.getElapsedTime() + "ms");
```

This is the most important single log line. If the shot misses, this tells you whether it was a velocity problem, a timing problem, or a turret problem.

### 5. GoalPos convergence — has the estimate stabilized?

The EMA filter takes ~35 frames (1.2 s) to converge. Log how far the raw measurement is from the current estimate:

```java
// In aiming(), after goalPos.update():
double posError = Math.sqrt(Math.pow(goalPos.a - rawX, 2) + Math.pow(goalPos.b - rawY, 2));
if (VERBOSE) telemetry.addData("GoalPos error", String.format("%.2f in", posError));
if (posError > 5.0) log("GoalPos high error: " + String.format("%.1f", posError) + " in");
```

---

## Summary: Minimal Viable Logging Checklist

If you only add five things, add these — they cover 90% of the "what went wrong" questions:

1. **Loop Hz** — field + display in `loop()` always
2. **State transitions** — log every call to `move()` with position and timestamp
3. **Camera lock time** — log when `gainSet` becomes true with elapsed time
4. **Shot moment** — log FW1, FW2, turret error, and `range` at the moment the stopper fires
5. **Timeout events** — log every time a state exits via timeout instead of via success condition
