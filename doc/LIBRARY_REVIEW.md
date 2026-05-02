# Library Review & Recommendations
**Date:** 2026-05-02

---

## Current Stack

| Library | Version | Role |
|---|---|---|
| FTC SDK (RobotCore, Hardware, Vision, etc.) | 11.0.0 | Core framework |
| Pedro Pathing (`com.pedropathing:ftc`) | 2.0.5 | Autonomous path following |
| Pedro Pathing Telemetry | 1.0.0 | Dashboard telemetry |
| FullPanels (`com.bylazar:fullpanels`) | 1.0.6 | Field visualizer UI |
| AndroidX AppCompat | 1.2.0 | Android support |

---

## Recommendation 1 — Limelight Vision Coprocessor
**Impact: High — offloads all vision computation from the Control Hub**

### What is the Limelight?

The **Limelight 3A** is a small camera module with its own built-in computer, designed specifically for FTC and FRC robots. Think of it as a dedicated "vision brain" that runs separately from the main Control Hub. Instead of asking the Control Hub to do everything — drive the robot, control the turret, track paths, AND process camera images all at once — the Limelight handles all the camera work on its own and simply tells the Control Hub the result ("the target is 48 inches away, 12 degrees to the left").

It costs approximately **$150** and mounts where your current webcam sits, powered by a single USB cable from the Control Hub.

---

### Why Does This Matter? The Problem with the Current Setup

The **REV Control Hub** — the orange box on the robot — has a single processor that has to do *everything* at the same time:

- Run Pedro Pathing (calculating motor powers 100+ times per second to follow the path)
- Control the turret (calculating the target angle and sending commands to the motor)
- Read the flywheel velocity and adjust motor power
- **Process camera frames** to detect the AprilTag and calculate where the goal is

This is like asking one person to simultaneously drive a car, solve a math problem, and read a map out loud. Each task takes time, and they compete with each other. When the processor is busy running Pedro Pathing during a fast autonomous move, camera frames may be processed more slowly or with more delay.

With Limelight, **the camera work is completely removed from the Control Hub's to-do list**. The Limelight's own processor handles all the image processing and sends the answer over a network connection. The Control Hub only needs to ask "what did you see?" — taking microseconds — rather than doing the processing itself.

---

### The Four Specific Problems Limelight Fixes in Your Code

**Problem 1: CPU Contention — the turret lags during fast movement**

Your turret needs to update its target angle approximately 30 times per second (`loop()` runs at ~30Hz) to smoothly track the goal while the robot is moving. However, Pedro Pathing also runs its path correction calculations on every loop. During a fast path segment — like driving from the intake position back to the outtake — Pedro Pathing is working hardest, and the camera processing that feeds the turret gets slower. This is why you may notice the turret being slightly off-target when arriving at the scoring position after a fast move.

With Limelight, Pedro Pathing and turret tracking no longer share CPU time. The turret gets fresh targeting data at full speed regardless of how hard the path follower is working.

---

**Problem 2: The `gainSet` Race Condition — the first shot may fire blind**

In your current code, every autonomous OpMode has this pattern:

```java
// In loop():
if (opmodeTimer.getElapsedTime() > 500 && !gainSet) {
    cameraControls();  // tries to set manual exposure
}

// In cameraControls():
if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
    exposureControl.setExposure(2, TimeUnit.MILLISECONDS);
    gainControl.setGain(100);
    gainSet = true;  // only NOW does vision targeting begin
}
```

The issue: the webcam connected to the Control Hub goes through a startup sequence when the OpMode begins (`OPENING_CAMERA_DEVICE → STARTING_STREAM → STREAMING`). This typically takes **1–3 seconds**. Until `gainSet` becomes `true`, the camera is running with auto-exposure, which produces blurry or washed-out images at match speed, and your `aiming()` method skips all tag detections:

```java
if (gainSet) {  // ← targeting skipped entirely until camera warms up
    for (AprilTagDetection detection : detectedTags) { ... }
}
```

This means the robot may reach the first scoring position (`SHOOTPRE`) before the camera has locked on, and the turret is only pointing at the hardcoded initial estimate `new GoalPos(147, 143, 15.5)`. If that estimate is even a few inches off, the first shot misses.

The Limelight is **always running** — it starts processing before the OpMode even begins, because it has its own power and processor. There is no warm-up delay, no `gainSet` flag needed, and no race condition. The first `limelight.getLatestResult()` call at the very start of autonomous already returns valid targeting data.

---

**Problem 3: `cameraControls()` complexity — exposure tuning in code is fragile**

The current code manually sets camera exposure to `2ms` and gain to `100` via the FTC SDK camera control API. These hardcoded values work in your practice space, but may be wrong at a competition venue with different lighting. Changing them requires editing code, rebuilding the APK (2–3 minutes), and redeploying.

The Limelight has a **built-in web dashboard** accessible from any laptop browser on the robot's WiFi. Exposure, gain, and all camera settings are adjusted in real time through the browser — no code change, no rebuild. You can tune it in the pit in 30 seconds.

---

**Problem 4: The `GoalPos` estimator is slow to converge — turret lags behind the true position**

Your `GoalPos` class uses an **Exponential Moving Average (EMA)** to smooth out noisy tag detections:

```java
// In GoalPos.update():
a = a * (1 - alpha) + a2 * alpha;  // alpha = 0.08
b = b * (1 - alpha) + b2 * alpha;
c = c * (1 - alpha) + c2 * alpha;
```

With `alpha = 0.08` and the camera running at ~30fps, it takes approximately **35 frames (about 1.2 seconds)** for the estimated goal position to reach 95% of the true position. This means for the first second of targeting after a new measurement arrives, the turret is pointing at a slightly wrong position.

The reason this smoothing exists is that the Control Hub's webcam-based AprilTag detection produces noisy pose estimates — the reported distance and angle jitter frame-to-frame due to lighting, motion blur, and limited CPU for image processing. EMA smooths out that noise, but at the cost of lag.

The Limelight's dedicated processor produces **significantly less noisy pose estimates** because it can run its full image pipeline without CPU competition. In practice this means you can use a higher alpha (faster convergence) or potentially remove the EMA smoothing from `GoalPos` entirely and use direct measurements. Either way the turret locks onto the target much faster.

---

### Code Change Summary

Replacing the current webcam setup with Limelight removes `initWebcam()`, `cameraControls()`, and the `gainSet` flag from every autonomous OpMode, and simplifies `aiming()`:

```java
// Current: ~40 lines of camera setup + gainSet management across every auto
private void initWebcam() { ... }          // ~30 lines
public void cameraControls() { ... }       // ~10 lines
private boolean gainSet = false;
if (opmodeTimer.getElapsedTime() > 500 && !gainSet) { cameraControls(); }

// With Limelight: camera is always ready, no setup needed in OpMode
limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.start();  // that's it

// In aiming():
LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
    camRange = tag.getSpatialX();   // direct 3D position in inches
    bearing  = Math.toRadians(tag.getTargetXDegrees() + ...);
    goalPos.update(0.3, xPos, yPos, bearing, elevation, camRange); // higher alpha, faster lock
}
```

**SDK dependency:**
```groovy
// Add to build.dependencies.gradle
implementation 'com.limelightvision:limelight-ftc:1.0.0'
```

**Hardware:** Limelight 3A (~$150). Mounts where your current webcam is. Powered via USB from the Control Hub. No additional wiring required.

---

## Recommendation 2 — Pedro Pathing: Switch to Predictive Braking
**Impact: High — ~15% faster autonomous paths**

Pedro Pathing 2.0.5 includes a Predictive Braking algorithm that replaces the manually-tuned translational and drive PIDFs. The Pedro docs state: *"A world-record autonomous was achieved, and many other teams' autos were sped up by ~15%, all while automatically tuning in a few minutes."*

**How it works:** Instead of a D term to prevent overshoot, the algorithm predicts how far the robot will slide during braking using measured deceleration constants, and begins braking at exactly the right moment. It maximizes deceleration without overshoot.

**What to remove from `Constants.java`:**
```java
// Remove:
.translationalPIDFCoefficients(...)
.secondaryTranslationalPIDFCoefficients(...)
.drivePIDFCoefficients(...)
.secondaryDrivePIDFCoefficients(...)
.useSecondaryTranslationalPIDF(true)
.useSecondaryDrivePIDF(true)
.centripetalScaling(0.0005)
```

**What to add:**
```java
.predictiveBrakingCoefficients(
    new PredictiveBrakingCoefficients(0.1, kLinear, kQuadratic)
)
.centripetalScaling(0)
```

**Tuning steps:**
1. Run `Tuning.java → Automatic → PredictiveBrakingTuner` — outputs `kLinear` and `kQuadratic` automatically
2. Run `LineTest` and tune `kP` between 0.05–0.3
3. Lower `tValueConstraint` from `0.99` to `0.95` in `PathConstraints` — Predictive Braking stops accurately so the path can end earlier

**Note:** Also lower `centripetalScaling` to `0` — Predictive Braking accounts for centripetal forces natively.

---

## Recommendation 3 — Update FullPanels 1.0.6 → 1.0.12
**Impact: Medium — visualizer fixes and compatibility**

The Pedro Pathing installation docs now specify `1.0.12`. Six minor versions behind means missing bug fixes in the field visualizer used during tuning sessions.

**Change in `build.dependencies.gradle`:**
```groovy
implementation 'com.bylazar:fullpanels:1.0.12'  // was 1.0.6
```

---

## Recommendation 4 — Add FTC Dashboard for Shooting System Tuning
**Impact: Medium — real-time flywheel/turret tuning without rebuilding**

Pedro Pathing's FullPanels covers drive path visualization, but the flywheel + turret + vision pipeline has no real-time tuning tool. Every adjustment to `p`, `f`, flap thresholds, or `toFWV` constants requires a full APK rebuild and redeploy (~2–3 minutes).

FTC Dashboard provides a web UI at `192.168.43.1:8080` with live-editable fields and telemetry graphs. The `@Config` annotation exposes any class's static fields as sliders:

```java
@Config
public class FlywheelTesting extends LinearOpMode {
    public static double p = 200;
    public static double f = 13.5;
    public static double FW1Target = 0;
    public static double FLAP_NEAR = 0.0;
    public static double FLAP_MID = 0.2;
    public static double FLAP_FAR = 0.24;
    // All editable live in the browser
}
```

**Add to `build.dependencies.gradle`:**
```groovy
repositories {
    maven { url = 'https://maven.brott.dev/' }
}
dependencies {
    implementation 'com.acmerobotics.dashboard:dashboard:0.4.16'
}
```

---

## Recommendation 5 — Use Pedro Pathing Ivy (Already Included)
**Impact: Medium — parallel drive + mechanism execution**

Ivy is already bundled in `com.pedropathing:ftc:2.0.5` but unused. It allows the flywheel to spin up in parallel while the robot is driving to the scoring position, rather than waiting until the robot arrives.

```java
// Current (sequential): drive → arrive → spin up → wait → shoot
// With Ivy (parallel):
parallel(
    follow(follower, Outtake1),
    run(() -> {
        flyWheel1.setVelocity(targetV);
        flyWheel2.setVelocity(targetV);
    })
)
```

No new dependency. See Pedro Pathing Ivy docs for full integration.

---

## Recommendation 6 — Evaluate SparkFun OTOS as Pinpoint Alternative
**Impact: Low-Medium — depends on current localization reliability**

Pedro Pathing supports the SparkFun OTOS (Optical Tracking Odometry Sensor) as an alternative to the GoBilda Pinpoint. OTOS uses optical flow rather than mechanical encoder pods.

| | GoBilda Pinpoint (current) | SparkFun OTOS |
|---|---|---|
| Mechanism | Two physical encoder pods | Optical flow sensor |
| Failure mode | Pod slip, wear, carpet snagging | Highly reflective or featureless floors |
| Wiring | I2C + pod cables | Single I2C cable |
| Robot weight | Heavier (pods + hardware) | Lighter |
| Drift | Low on good surfaces | Very low on most FTC floors |

**Recommendation:** Only evaluate if localization drift is observed mid-auto (robot missing intake positions on the 3rd or 4th cycle). If Pinpoint is tracking reliably, no change is warranted.

---

## Not Recommended

### AndroidX AppCompat 1.2.0
Leave at current version. The FTC SDK pins to `1.2.0` due to its `targetSdkVersion 28` constraint. Upgrading independently risks breaking SDK UI components with no performance benefit.

---

## Priority Summary

| # | Change | Effort | Expected Gain |
|---|--------|--------|---------------|
| 1 | Add Limelight 3A coprocessor | Hardware + 1–2 days integration | Vision latency, Control Hub CPU freed |
| 2 | Switch to Predictive Braking | 1–2 hour tuning session | ~15% faster auto paths |
| 3 | Update FullPanels to 1.0.12 | 5 minutes | Visualizer fixes |
| 4 | Add FTC Dashboard | 1 hour setup | Real-time shooting system tuning |
| 5 | Use Ivy for parallel execution | Code refactor | 0.5–1s saved per shoot cycle |
| 6 | Evaluate OTOS | 1 test session | Localization reliability |

**Recommended order of implementation:**
1. Fix all P0 build errors first (see `CODE_REVIEW.md`)
2. Predictive Braking (quick win, no hardware cost)
3. FTC Dashboard (speeds up all subsequent tuning)
4. Limelight (biggest vision improvement, plan for next build window)
5. Ivy (refactor once build is stable)
